import asyncio
import json
import rclpy
from rclpy.node import Node
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
from fastapi import FastAPI
from std_msgs.msg import String
from functools import partial
from .routes import get_routers
from contextlib import asynccontextmanager

# Global node_manager for access from routes
node_manager = None

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('http_node')
        self.get_logger().info("HttpNode has been initialized")
        
        self.lock = threading.Lock()
        self._publishers = []
        self.service_clients = {}
        self.websocket_client = None
        self.subscribed_topics = []  # TODO: create initialize list of subscribers
        self.subscribers = [] # TODO: make this dynamic with a dynamic callback that can send to multiple websockets
        self._message_queue = asyncio.Queue()
        self._background_tasks = set()
        
    def init_subscribers(self):
        """Initialize subscribers for topics"""
        # TODO: change the implementation to support adding subscribers dynamically
        for topic in set(self.subscribed_topics):
            self.get_logger().info(f"subscribing to topic: {topic}")
            self.subscribers.append(
                self.create_subscription(
                    String,
                    topic,
                    partial(self.message_callback, topic=topic),
                    10,
                )
            )

    def message_callback(self, message, topic):
        """Synchronous callback that queues messages for async processing"""
        if self.websocket_client:
            self._message_queue.put_nowait({
                "topic": topic,
                "message": message.data
            })
    
    async def process_message_queue(self):
        """Async task to process queued messages"""
        while True:
            try:
                message = await self._message_queue.get()
                if self.websocket_client:
                    await self.websocket_client.send_text(json.dumps(message))
                self._message_queue.task_done()
            except Exception as e:
                self.get_logger().error(f"Error processing message: {str(e)}")
            await asyncio.sleep(0.01)  # Prevent CPU spinning
    

################## Methods for Adding Publishers and Publishing Messages ##################
    def add_publisher(self, topic_name, message_type):
        with self.lock:
            publisher = self.create_publisher(message_type, topic_name, 10)
            self._publishers.append(publisher)

    def publish_message(self, topic_name, message):
        with self.lock:
            for publisher in self._publishers:
                if publisher.topic_name == topic_name:
                    msg = String()
                    msg.data = message
                    publisher.publish(msg)
                    return
            self.get_logger().warn("Publisher for topic %s not found", topic_name)
    
################## Methods for Adding Service Clients and Calling Services ##################    
    def add_service_client(self, service_name, service_type):
        with self.lock:
            self.service_clients[service_name] = self.create_client(service_type, service_name)

    async def call_service(self, service_name, request):
        with self.lock:
            if service_name in self.service_clients:
                future = self.service_clients[service_name].call_async(request)
                await future
                return future.result()
            else:
                self.get_logger().warn("Service client for service %s not found", service_name)
                return None

class NodeManager:
    def __init__(self):
        self.node = None

    def init_ros_node(self):
        rclpy.init()
        self.node = CommunicationNode()
        self.node.init_subscribers()
        rclpy.spin(self.node)
        rclpy.shutdown()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: init ros node and start it in a separate thread
    global node_manager
    node_manager = NodeManager()
    thread = threading.Thread(target=node_manager.init_ros_node)
    thread.daemon = True
    thread.start()
    
    # Wait for node to be initialized
    while node_manager.node is None:
        await asyncio.sleep(0.1)
    
    # Create and manage background task for processing message queue
    task = asyncio.create_task(node_manager.node.process_message_queue())
    node_manager.node._background_tasks.add(task)
    task.add_done_callback(node_manager.node._background_tasks.discard)
    
    yield  # At this point, the application is ready to serve requests
    
    # Shutdown/cleanup
    for task in node_manager.node._background_tasks:
        task.cancel()
    await asyncio.gather(*node_manager.node._background_tasks, return_exceptions=True)

def create_app():
    app = FastAPI(lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    # include all routers
    app.include_router(get_routers())

    return app

def main(args=None):
    app = create_app()
    uvicorn.run(app, host="127.0.0.1", port=8080)

if __name__ == '__main__':
    main()