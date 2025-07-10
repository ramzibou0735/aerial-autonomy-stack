import gi
import numpy as np
import sys
import signal

# GStreamer Python bindings
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Gazebo Transport Python bindings
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# --- Configuration ---
GZ_TOPIC = "/camera"
DEST_HOST = "42.42.1.1" # IP of the container that will receive the stream
DEST_PORT = 5600

# Global variables
pipeline = None
appsrc = None
main_loop = None

def on_new_gz_frame(msg: Image):
    """Callback function called by Gazebo Transport when a new frame arrives."""
    global appsrc, pipeline

    if appsrc is None:
        return

    # On the first frame, configure the GStreamer pipeline caps
    if appsrc.get_property('caps') is None:
        print(f"First frame received. Configuring GStreamer pipeline for {msg.width}x{msg.height}...")
        caps_str = f"video/x-raw,format=RGB,width={msg.width},height={msg.height},framerate=30/1"
        caps = Gst.Caps.from_string(caps_str)
        appsrc.set_property('caps', caps)

    # Create a GStreamer buffer from the raw image data
    # The 'new_wrapped' method avoids copying the data, which is efficient
    buffer = Gst.Buffer.new_wrapped(msg.data)
    
    # Push the buffer into the appsrc element. GStreamer handles the rest.
    retval = appsrc.emit('push-buffer', buffer)
    if retval != Gst.FlowReturn.OK:
        print("Error pushing buffer to GStreamer")

def main():
    """Main function to set up GStreamer and Gazebo subscription."""
    global pipeline, appsrc, main_loop

    # --- Initialize GStreamer ---
    Gst.init(None)

    # This is the GStreamer sender pipeline description
    pipeline_str = (
        f"appsrc name=py_source ! videoconvert ! x264enc tune=zerolatency ! "
        f"rtph264pay ! udpsink host={DEST_HOST} port={DEST_PORT}"
    )
    
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name('py_source')

    # --- Initialize Gazebo Transport Subscriber ---
    node = Node()
    if not node.subscribe(Image, GZ_TOPIC, on_new_gz_frame):
        print(f"Error subscribing to topic [{GZ_TOPIC}]")
        return

    # Start the GStreamer pipeline
    pipeline.set_state(Gst.State.PLAYING)
    print(f"GStreamer pipeline started. Streaming to udp://{DEST_HOST}:{DEST_PORT}...")
    print(f"Subscribed to Gazebo topic [{GZ_TOPIC}]. Waiting for frames...")

    # Run the main loop to keep the script alive
    main_loop = GLib.MainLoop()
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, lambda s, f: main_loop.quit())
    
    main_loop.run()

    # Clean up
    print("Shutting down...")
    pipeline.set_state(Gst.State.NULL)


if __name__ == '__main__':
    main()