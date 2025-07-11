import argparse
import gi
import signal
gi.require_version('Gst', '1.0')

from gi.repository import Gst, GLib
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

pipeline = None
appsrc = None
main_loop = None

def on_new_gz_frame(msg: Image):
    global appsrc, pipeline

    if appsrc is None:
        return

    if appsrc.get_property('caps') is None:
        print(f"First frame received. Configuring GStreamer pipeline for {msg.width}x{msg.height}...")
        caps_str = f"video/x-raw,format=RGB,width={msg.width},height={msg.height},framerate=30/1"
        caps = Gst.Caps.from_string(caps_str)
        appsrc.set_property('caps', caps)

    buffer = Gst.Buffer.new_wrapped(msg.data)
    
    retval = appsrc.emit('push-buffer', buffer)
    if retval != Gst.FlowReturn.OK:
        print("Error pushing buffer to GStreamer")

def main():
    global pipeline, appsrc, main_loop

    parser = argparse.ArgumentParser(description="Bridge a Gazebo camera topic to a GStreamer UDP stream.")
    parser.add_argument('--gz_topic', default="/camera", help="Gazebo topic to subscribe to.")
    parser.add_argument('--ip', default="42.42.1.1", help="Destination host for the GStreamer stream.")
    parser.add_argument('--port', type=int, default=5600, help="Destination port for the GStreamer stream.")
    args = parser.parse_args()

    Gst.init(None)

    pipeline = Gst.parse_launch(
        f"appsrc name=py_source ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host={args.ip} port={args.port}"
        )
    appsrc = pipeline.get_by_name('py_source')

    node = Node()
    if not node.subscribe(Image, args.gz_topic, on_new_gz_frame):
        print(f"Error subscribing to topic [{args.gz_topic}]. Ensure Gazebo is running and the topic exists.")
        return

    pipeline.set_state(Gst.State.PLAYING)
    print(f"GStreamer pipeline started. Streaming to udp://{args.ip}:{args.port}...")
    print(f"Subscribed to Gazebo topic [{args.gz_topic}]. Waiting for frames...")

    main_loop = GLib.MainLoop() 
    signal.signal(signal.SIGINT, lambda s, f: main_loop.quit()) # Handle Ctrl + c
    main_loop.run()

    print("Shutting down...")
    pipeline.set_state(Gst.State.NULL) # Clean up

if __name__ == '__main__':
    main()
