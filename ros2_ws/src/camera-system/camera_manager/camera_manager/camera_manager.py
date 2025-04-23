import rclpy
from rclpy.node import Node
from interfaces.srv import RequestStream, SetAnswer, AddIceCandidate, RemovePeer
from interfaces.msg import WebRTCMessage
import json
import gi
import os
from pathlib import Path
import asyncio
from typing import Dict, Optional
from concurrent.futures import ThreadPoolExecutor
import threading

# GStreamer imports
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib
class CameraStream:
    def __init__(self, device_path: str, port: int, codec='vp8'):
        self.device_path = device_path
        self.port = port
        self.codec = codec.lower()
        self.pipeline = None
        self.webrtc = None
        self.peer_connections = {}
        self.pipeline_ready = False
        self.error_message = None

    def create_pipeline(self):
        """Create and set up the GStreamer pipeline"""
        try:
            # Initialize GStreamer
            Gst.init(None)

            # Create pipeline string based on codec
            # Use MJPG format from camera for better performance
            if self.codec == 'vp8':
                pipeline_str = (
                    f'v4l2src device={self.device_path} ! '
                    'image/jpeg,width=640,height=480,framerate=30/1 ! '
                    'jpegdec ! videoconvert ! videoscale ! '
                    'video/x-raw,format=YUY2,width=640,height=480 ! '
                    'vp8enc deadline=1 cpu-used=4 target-bitrate=2000000 error-resilient=true ! '
                    'rtpvp8pay ! '
                    'webrtcbin name=webrtc bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302'
                )
            else:  # h264
                pipeline_str = (
                    f'v4l2src device={self.device_path} ! '
                    'image/jpeg,width=640,height=480,framerate=30/1 ! '
                    'jpegdec ! videoconvert ! videoscale ! '
                    'video/x-raw,format=YUY2,width=640,height=480 ! '
                    'x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 '
                    'bitrate=2000 threads=4 ! '
                    'video/x-h264,stream-format=byte-stream,profile=baseline ! '
                    'h264parse ! '
                    'rtph264pay config-interval=1 ! '
                    'webrtcbin name=webrtc bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302'
                )

            print(f"Creating pipeline with string: {pipeline_str}")
            
            # Create pipeline from string
            self.pipeline = Gst.parse_launch(pipeline_str)
            if not self.pipeline:
                raise Exception("Failed to create pipeline")

            # Get webrtcbin by name
            self.webrtc = self.pipeline.get_by_name('webrtc')
            if not self.webrtc:
                raise Exception("Failed to get webrtcbin element")

            # Connect webrtc signals
            self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
            self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)

            # Add debugging probes
            def on_buffer_probe(pad, info):
                buffer = info.get_buffer()
                print(f"Buffer flowing through {pad.get_parent_element().get_name()}, size: {buffer.get_size()}")
                return Gst.PadProbeReturn.OK

            # Add probes to all elements
            def add_probes(element):
                src_pad = element.get_static_pad('src')
                if src_pad:
                    src_pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer_probe)

            # Iterate through all elements and add probes
            iterator = self.pipeline.iterate_elements()
            while True:
                result = iterator.next()
                if result[0] == Gst.IteratorResult.DONE:
                    break
                if result[0] == Gst.IteratorResult.OK:
                    element = result[1]
                    add_probes(element)

            # Set up bus monitoring
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self.on_bus_message)

            # Start the pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("Failed to set pipeline to PLAYING state")

            print(f"Pipeline successfully created and started with {self.codec} codec")
            self.pipeline_ready = True

        except Exception as e:
            self.error_message = str(e)
            print(f"Error creating pipeline: {e}")
            self.cleanup()
            raise

    def on_bus_message(self, bus, message):
        """Handle pipeline bus messages with more detailed logging"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Pipeline Error: {err.message}")
            print(f"Debug details: {debug}")
            print(f"Source element: {message.src.get_name()}")
            
            # Get more detailed element state information
            if message.src:
                state = message.src.get_state(0)
                print(f"Element state: {state[1].value_name}")
            
            self.error_message = err.message
            self.pipeline_ready = False
            self.cleanup()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"Pipeline Warning: {err.message}")
            print(f"Debug details: {debug}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                print(f"Pipeline state changed from {old_state.value_nick} to {new_state.value_nick}")
                
                # If entering PLAYING state, mark pipeline as ready
                if new_state == Gst.State.PLAYING:
                    self.pipeline_ready = True

    def on_negotiation_needed(self, element):
        if not self.pipeline_ready:
            print("Pipeline not ready, skipping negotiation")
            return
            
        print("Negotiation needed, creating offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"Error in negotiation: {e}")

    def on_offer_created(self, promise, element):
        print("Offer created, setting local description...")
        try:
            promise.wait()
            reply = promise.get_reply()
            offer = reply.get_value('offer')
            if not offer:
                raise Exception("No offer in promise reply")
            
            sdp = offer.sdp.as_text()
            print(f"Generated SDP:\n{sdp}")
            
            element.emit('set-local-description', offer, None)
            self.pending_offer = {
                'type': 'offer',
                'sdp': sdp
            }
            print("Local description set")
        except Exception as e:
            print(f"Error creating offer: {e}")
            self.error_message = str(e)
            self.cleanup()

    def on_ice_candidate(self, webrtc, mlineindex, candidate):
        if not hasattr(self, 'pending_ice_candidates'):
            self.pending_ice_candidates = []
        self.pending_ice_candidates.append({
            'candidate': candidate,
            'sdpMLineIndex': mlineindex
        })
        print(f"ICE candidate collected: {candidate}")

    def cleanup(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.webrtc = None
        self.pipeline_ready = False






class CameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager')
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Load camera configuration
        self.config = self._load_config()
        
        # Dictionary to store active camera streams
        self.cameras: Dict[str, CameraStream] = {}

        # Create camera order mapping
        self.camera_order = {
            camera_id: index 
            for index, camera_id in enumerate(self.config['CAMERAS'].keys())
        }
        
        # GLib main loop for GStreamer
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run)
        self.loop_thread.daemon = True
        self.loop_thread.start()
        
        # Create services
        self.create_services()
        
        # WebRTC message publisher
        self.webrtc_pub = self.create_publisher(
            WebRTCMessage,
            '/camera/webrtc_messages',
            10
        )
        
        self.get_logger().info('Camera Manager initialized')

    def _load_config(self) -> dict:
        """Load camera configuration from JSON file"""
        current_path = Path(__file__).resolve()
        current_path_str = str(current_path)
        ros2_ws_index = current_path_str.rfind("ros2_ws/")
        if ros2_ws_index == -1:
            raise FileNotFoundError("ros2_ws directory not found in the path.")
        
        base_path = current_path_str[:ros2_ws_index + len("ros2_ws/")]
        config_path = Path(base_path).parent / 'cams' / 'cameras.json'
        
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            return json.load(f)
        

    def create_services(self):
        """Create ROS2 services"""
        self.srv_request_stream = self.create_service(
            RequestStream,
            '/camera/request_stream',
            self._handle_stream_request
        )
        
        self.srv_set_answer = self.create_service(
            SetAnswer,
            '/camera/set_answer',
            self._handle_set_answer
        )
        
        self.srv_add_ice = self.create_service(
            AddIceCandidate,
            '/camera/add_ice_candidate',
            self._handle_add_ice_candidate
        )
        
        self.srv_remove_peer = self.create_service(
            RemovePeer,
            '/camera/remove_peer',
            self._handle_remove_peer
        )

    def _get_camera_port(self, camera_id: str) -> int:
        """Get unique port for camera based on BASE_PORT and camera order"""
        if camera_id not in self.camera_order:
            raise ValueError(f"Camera ID {camera_id} not found in configuration")
            
        return self.config['BASE_PORT'] + self.camera_order[camera_id]

    def _handle_stream_request(self, request, response):
        """Handle incoming stream requests"""
        self.get_logger().info(f"Received stream request: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            codec = request.codec.lower()  # Get the requested codec
            
            if camera_id not in self.config['CAMERAS']:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            # Get or create camera stream
            if camera_id not in self.cameras:
                device_path = self.config['CAMERAS'][camera_id]
                port = self._get_camera_port(camera_id)
                
                camera_stream = CameraStream(device_path, port, codec)  # Pass the codec
                try:
                    camera_stream.create_pipeline()
                except Exception as e:
                    response.success = False
                    response.error_message = str(e)
                    return response
                
                self.cameras[camera_id] = camera_stream
            
            camera_stream = self.cameras[camera_id]
            
            # Check if pipeline is ready
            if not camera_stream.pipeline_ready:
                response.success = False
                response.error_message = camera_stream.error_message or "Pipeline not ready"
                return response
            
            # Wait for offer with timeout
            start_time = self.get_clock().now()
            timeout = 5.0  # 5 seconds timeout
            
            while not hasattr(camera_stream, 'pending_offer'):
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    response.success = False
                    response.error_message = "Timeout waiting for WebRTC offer"
                    return response
                    
                if not camera_stream.pipeline_ready:
                    response.success = False
                    response.error_message = camera_stream.error_message or "Pipeline failed during offer creation"
                    return response
                    
                self.get_logger().info(f"Waiting for offer from camera {camera_id}")
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Get offer
            offer = camera_stream.pending_offer
            delattr(camera_stream, 'pending_offer')
            
            response.success = True
            response.offer_sdp = offer['sdp']
            response.offer_type = offer['type']
            
            # Store peer connection association
            camera_stream.peer_connections[peer_id] = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling stream request: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_set_answer(self, request, response):
        """Handle incoming answers"""
        self.get_logger().info(f"Received answer: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            
            if camera_id not in self.cameras:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            camera_stream = self.cameras[camera_id]
            
            # Set remote description
            answer = GstWebRTC.WebRTCSessionDescription.new(
                GstWebRTC.WebRTCSDPType.ANSWER,
                GstSdp.SDPMessage.parse_buffer(request.answer_sdp.encode())
            )
            promise = Gst.Promise.new()
            camera_stream.webrtc.emit('set-remote-description', answer, promise)
            promise.wait()
            
            # Send any pending ICE candidates
            if hasattr(camera_stream, 'pending_ice_candidates'):
                for candidate in camera_stream.pending_ice_candidates:
                    self._publish_webrtc_message(
                        peer_id,
                        camera_id,
                        'ice-candidate',
                        candidate
                    )
                delattr(camera_stream, 'pending_ice_candidates')
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling set answer: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_add_ice_candidate(self, request, response):
        """Handle incoming ICE candidates"""
        self.get_logger().info(f"Received ICE candidate: {request}")
        try:
            camera_id = request.camera_id
            
            if camera_id not in self.cameras:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            camera_stream = self.cameras[camera_id]
            
            # Add ICE candidate
            camera_stream.webrtc.emit(
                'add-ice-candidate',
                request.candidate_sdp_mline_index,
                request.candidate_sdp
            )
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling ICE candidate: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_remove_peer(self, request, response):
        """Handle peer removal requests"""
        self.get_logger().info(f"Removing peer: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            
            if camera_id in self.cameras:
                camera_stream = self.cameras[camera_id]
                
                # Remove peer from camera stream
                if peer_id in camera_stream.peer_connections:
                    del camera_stream.peer_connections[peer_id]
                
                # If no more peers, cleanup camera stream
                if not camera_stream.peer_connections:
                    camera_stream.cleanup()
                    del self.cameras[camera_id]
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling peer removal: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _publish_webrtc_message(self, peer_id: str, camera_id: str, message_type: str, data: dict):
        """Publish WebRTC message to the client"""
        self.get_logger().info(f"Publishing WebRTC message: {message_type}")
        msg = WebRTCMessage()
        msg.peer_id = peer_id
        msg.camera_id = camera_id
        msg.type = message_type
        msg.data = json.dumps(data)
        self.webrtc_pub.publish(msg)

    def cleanup(self):
        """Cleanup all camera streams"""
        for camera_stream in self.cameras.values():
            camera_stream.cleanup()
        self.cameras.clear()
        
        if self.loop.is_running():
            self.loop.quit()
        
        if self.loop_thread.is_alive():
            self.loop_thread.join()

def main(args=None):
    rclpy.init(args=args)
    camera_manager = CameraManager()
    try:
        rclpy.spin(camera_manager)
    finally:
        camera_manager.cleanup()
        camera_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()