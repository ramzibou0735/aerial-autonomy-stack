#!/usr/bin/env python3
"""
Batch import .ulog files to Flight Review database
This mimics the upload.py process from flight_review
"""

import os
import sys
import glob
import sqlite3
import datetime
import uuid
import binascii
import argparse
import shutil
from pathlib import Path
import asyncio
from tornado.ioloop import IOLoop
import threading

# Add flight_review paths to Python path
FLIGHT_REVIEW_PATH = os.path.expanduser('/git/flight_review')
sys.path.append(os.path.join(FLIGHT_REVIEW_PATH, 'app'))
sys.path.append(os.path.join(FLIGHT_REVIEW_PATH, 'app/plot_app'))

try:
    from pyulog import ULog
    from pyulog.px4 import PX4ULog
    from config import get_db_filename
    from helper import get_total_flight_time, get_log_filename, load_ulog_file, \
        get_airframe_name, ULogException
    from db_entry import DBVehicleData, DBData
    from tornado_handlers.common import generate_db_data_from_log_file
    from overview_generator import generate_overview_img_from_id
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this from the flight_review environment")
    print("You may need to use/activate the virtual environment")
    sys.exit(1)


def get_configured_log_files_directory():
    """
    Get the configured log files directory
    Falls back to default if not configured
    """
    try:
        from config import get_configured_log_files_directory as get_dir
        return get_dir()
    except ImportError:
        default_path = os.path.join(FLIGHT_REVIEW_PATH, 'data', 'log_files')
        if not os.path.exists(default_path):
            os.makedirs(default_path, exist_ok=True)
        return default_path


def update_vehicle_db_entry(cur, ulog, log_id, vehicle_name=''):
    """
    Update the Vehicle DB entry (adapted from upload.py)
    """
    vehicle_data = DBVehicleData()
    if 'sys_uuid' in ulog.msg_info_dict:
        vehicle_data.uuid = ulog.msg_info_dict['sys_uuid']
        
        if vehicle_name == '':
            cur.execute('select Name from Vehicle where UUID = ?', [vehicle_data.uuid])
            db_tuple = cur.fetchone()
            if db_tuple is not None:
                vehicle_data.name = db_tuple[0]
        else:
            vehicle_data.name = vehicle_name
        
        vehicle_data.log_id = log_id
        flight_time = get_total_flight_time(ulog)
        if flight_time is not None:
            vehicle_data.flight_time = flight_time
        
        # update or insert the DB entry
        cur.execute('insert or replace into Vehicle (UUID, LatestLogId, Name, FlightTime)'
                    'values (?, ?, ?, ?)',
                    [vehicle_data.uuid, vehicle_data.log_id, vehicle_data.name,
                     vehicle_data.flight_time])
    return vehicle_data


def extract_additional_info(ulog):
    """
    Extract additional information from ULog for database (adapted from upload.py)
    """
    info = {}
    
    try:
        px4_ulog = PX4ULog(ulog)
        info['mav_type'] = px4_ulog.get_mav_type()
    except:
        info['mav_type'] = ''
    
    # Extract airframe info
    try:
        airframe_name_tuple = get_airframe_name(ulog)
        if airframe_name_tuple is not None:
            airframe_name, airframe_id = airframe_name_tuple
            if len(airframe_name) == 0:
                info['airframe'] = str(airframe_id)
            else:
                info['airframe'] = airframe_name
    except:
        info['airframe'] = ''
    
    # Extract hardware info
    info['hardware'] = ''
    if 'ver_hw' in ulog.msg_info_dict:
        info['hardware'] = ulog.msg_info_dict['ver_hw']
    
    # Extract software version
    info['software'] = ''
    branch_info = ''
    if 'ver_sw_branch' in ulog.msg_info_dict:
        branch_info = ' (branch: '+ulog.msg_info_dict['ver_sw_branch']+')'
    if 'ver_sw' in ulog.msg_info_dict:
        info['software'] = ulog.msg_info_dict['ver_sw'] + branch_info
    
    # Extract UUID
    info['uuid'] = ''
    if 'sys_uuid' in ulog.msg_info_dict and info['hardware'] != 'SITL':
        info['uuid'] = ulog.msg_info_dict['sys_uuid']
    
    return info


def process_ulog_file(ulog_path, con, source='script_import', 
                      description='', public=True, generate_db_entry=True,
                      allow_for_analysis=True, generate_overview=True):
    """
    Process a single ulog file and add it to the database
    
    Args:
        ulog_path: Path to the .ulog file
        con: SQLite database connection
        source: Source identifier for the upload
        description: Description for the log
        public: Whether the log should be public
        generate_db_entry: Whether to generate additional DB analysis data
        allow_for_analysis: Whether to allow the log for analysis
        generate_overview: Whether to generate overview image
    
    Returns:
        log_id if successful, None if failed
    """
    
    try:
        # First check if it's a valid ULog file
        with open(ulog_path, 'rb') as f:
            header = f.read(len(ULog.HEADER_BYTES))
            if header != ULog.HEADER_BYTES:
                print(f"Error: {ulog_path} is not a valid ULog file")
                return None
        
        # Generate unique log ID
        log_id = str(uuid.uuid4())
        
        # Determine destination path
        log_files_dir = get_configured_log_files_directory()
        if not os.path.exists(log_files_dir):
            os.makedirs(log_files_dir, exist_ok=True)
        
        new_file_name = get_log_filename(log_id)
        
        # Copy file to the logs directory
        # print(f"Copying {os.path.basename(ulog_path)} to {new_file_name}")
        shutil.copy2(ulog_path, new_file_name)
        
        # Load the ULog file for metadata extraction
        try:
            ulog = load_ulog_file(new_file_name)
        except Exception as e:
            print(f"Warning: Could not fully load ULog file: {e}")
            # Try basic ULog loading
            ulog = ULog(new_file_name)
        
        # Generate a token for potential deletion
        token = str(binascii.hexlify(os.urandom(16)), 'ascii')
        
        # Extract additional info
        additional_info = extract_additional_info(ulog)
        
        # Prepare database values
        title = ''
        original_filename = os.path.basename(ulog_path)
        obfuscated = 0
        email = ''
        wind_speed = -1
        rating = ''
        feedback = ''
        upload_type = 'flightreport' if public else 'personal'  # Set as flightreport for public logs
        video_url = ''
        error_labels = ''
        is_public = 1 if public else 0
        allow_analysis = 1 if allow_for_analysis else 0
        
        # Insert into database
        cur = con.cursor()
        
        # Check if log already exists (shouldn't happen with UUID, but just in case)
        cur.execute('SELECT Id FROM Logs WHERE Id = ?', [log_id])
        if cur.fetchone():
            print(f"Warning: Log ID {log_id} already exists (this should not happen)")
            os.remove(new_file_name)
            return None
        
        # Insert the log entry
        cur.execute(
            'INSERT INTO Logs (Id, Title, Description, '
            'OriginalFilename, Date, AllowForAnalysis, Obfuscated, '
            'Source, Email, WindSpeed, Rating, Feedback, Type, '
            'videoUrl, ErrorLabels, Public, Token) VALUES '
            '(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
            [log_id, title, description, original_filename,
             datetime.datetime.now(), allow_analysis,
             obfuscated, source, email, wind_speed, rating,
             feedback, upload_type, video_url, error_labels, is_public, token])
        
        # Update vehicle database entry
        vehicle_data = update_vehicle_db_entry(cur, ulog, log_id)
        
        con.commit()
        
        # Generate additional DB data if requested (for analysis)
        # This MUST be done after the initial commit
        if generate_db_entry and is_public:
            try:
                # print(f"  Generating analysis data...")
                generate_db_data_from_log_file(log_id, con)
                con.commit()
                # print(f"  Generated analysis data for {original_filename}")
            except Exception as e:
                print(f"  Warning: Could not generate analysis data: {e}")
                import traceback
                traceback.print_exc()
        
        # Generate overview image if requested
        if generate_overview and is_public:
            try:
                # print(f"  Generating overview image...")
                # The overview generator expects to run in a Tornado IOLoop, create a simple wrapper to run it
                def generate_overview():
                    try:
                        generate_overview_img_from_id(log_id)
                    except Exception as e:
                        print(f"  Warning: Could not generate overview image: {e}")
                
                # Run in a separate thread to avoid blocking
                thread = threading.Thread(target=generate_overview)
                thread.start()
                thread.join(timeout=10)  # Wait max 10 seconds
                
                if thread.is_alive():
                    print(f"  Warning: Overview generation timed out")
                # else:
                #     print(f"  Generated overview image")
            except Exception as e:
                print(f"  Warning: Could not generate overview image: {e}")
        
        print(f"Successfully imported: {original_filename}, URL: http://42.42.1.99:5006/plot_app?log={log_id}")        
        return log_id
        
    except ULogException as e:
        print(f"Error: Failed to parse {ulog_path}: {e}")
        if 'new_file_name' in locals() and os.path.exists(new_file_name):
            os.remove(new_file_name)
        return None
    except Exception as e:
        print(f"Error processing {ulog_path}: {e}")
        import traceback
        traceback.print_exc()
        if 'new_file_name' in locals() and os.path.exists(new_file_name):
            os.remove(new_file_name)
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Batch import .ulog files to Flight Review database')
    parser.add_argument('path', 
                       help='Path to directory containing .ulog files or single .ulog file')
    parser.add_argument('--recursive', '-r', action='store_true',
                       help='Recursively search for .ulog files in subdirectories')
    parser.add_argument('--description', '-d', default='',
                       help='Description for all imported logs')
    parser.add_argument('--source', '-s', default='script_import',
                       help='Source identifier (default: script_import)')
    parser.add_argument('--public', action='store_true',
                       help='Make logs public (visible in browse)')
    parser.add_argument('--private', action='store_true',
                       help='Make logs private (not visible in browse)')
    parser.add_argument('--no-analysis', action='store_true',
                       help='Skip generating analysis data (faster import)')
    parser.add_argument('--no-overview', action='store_true',
                       help='Skip generating overview images')
    parser.add_argument('--flight-review-path', default=None,
                       help='Path to flight_review installation')
    
    args = parser.parse_args()
    
    # Handle public/private flags
    is_public = True  # Default to public
    if args.private:
        is_public = False
    elif args.public:
        is_public = True
    
    # Update path if specified
    if args.flight_review_path:
        global FLIGHT_REVIEW_PATH
        FLIGHT_REVIEW_PATH = args.flight_review_path
        sys.path.insert(0, os.path.join(FLIGHT_REVIEW_PATH, 'app'))
        sys.path.insert(0, os.path.join(FLIGHT_REVIEW_PATH, 'app/plot_app'))
    
    # Find all .ulog files
    ulog_files = []
    
    if os.path.isfile(args.path):
        if args.path.endswith('.ulg') or args.path.endswith('.ulog'):
            ulog_files = [args.path]
        else:
            print(f"Error: {args.path} is not a .ulog or .ulg file")
            sys.exit(1)
    elif os.path.isdir(args.path):
        if args.recursive:
            ulog_files = glob.glob(os.path.join(args.path, '**/*.ulg'), recursive=True)
            ulog_files += glob.glob(os.path.join(args.path, '**/*.ulog'), recursive=True)
        else:
            ulog_files = glob.glob(os.path.join(args.path, '*.ulg'))
            ulog_files += glob.glob(os.path.join(args.path, '*.ulog'))
    else:
        print(f"Error: Path {args.path} does not exist")
        sys.exit(1)
    
    if not ulog_files:
        print("No .ulog or .ulg files found")
        sys.exit(1)
    
    # print(f"Found {len(ulog_files)} ulog file(s)")
    # if is_public:
    #     print("Files will be marked as PUBLIC (visible in browse)")
    # else:
    #     print("Files will be marked as PRIVATE")
    
    # Connect to database
    try:
        db_filename = get_db_filename()
        # print(f"Using database: {db_filename}")
        con = sqlite3.connect(db_filename)
    except Exception as e:
        print(f"Error connecting to database: {e}")
        print("Make sure Flight Review is properly configured")
        sys.exit(1)
    
    # Process each file
    success_count = 0
    log_ids = []
    
    for i, ulog_file in enumerate(ulog_files, 1):
        print(f"[{i}/{len(ulog_files)}] Processing {os.path.basename(ulog_file)}...")
        log_id = process_ulog_file(
            ulog_file, 
            con,
            source=args.source,
            description=args.description,
            public=is_public,
            generate_db_entry=not args.no_analysis,
            allow_for_analysis=True,
            generate_overview=not args.no_overview
        )
        if log_id:
            success_count += 1
            log_ids.append(log_id)
    
    # Close database connection
    con.close()
    
    # print(f"\n{'='*50}")
    # print(f"Import complete: {success_count}/{len(ulog_files)} files successfully imported")
    
    # if success_count > 0:
    #     print(f"\nYou can view the imported logs at:")
    #     if is_public:
    #         print(f"  Browse page: http://42.42.1.99:5006/browse")
    #     print(f"\nDirect links to imported logs:")
    #     for log_id in log_ids[:5]:  # Show first 5
    #         print(f"  http://42.42.1.99:5006/plot_app?log={log_id}")
    #     if len(log_ids) > 5:
    #         print(f"  ... and {len(log_ids) - 5} more")
        
    #     if not args.no_overview and is_public:
    #         print(f"\nNote: Overview images are being generated in the background.")
    #         print(f"Refresh the browse page in a few seconds to see the map thumbnails.")


if __name__ == "__main__":
    main()
