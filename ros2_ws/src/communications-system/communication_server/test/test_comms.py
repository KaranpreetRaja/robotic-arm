import unittest
import asyncio
import websockets
import requests
import json
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import aiohttp

class TestSubscriberNode(Node):
    def __init__(self):
        super().__init__('test_subscriber_node')
        self.received_messages = {}
        
    def create_test_subscription(self, topic_name):
        self.received_messages[topic_name] = []
        return self.create_subscription(
            String,
            topic_name,
            lambda msg, topic=topic_name: self.message_callback(msg, topic),
            10
        )
        
    def message_callback(self, msg, topic):
        self.received_messages[topic].append(msg.data)

class TestHttpNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
        # init test subscriber node and spin it in a separate thread
        cls.test_node = TestSubscriberNode()
        cls.spin_thread = threading.Thread(target=cls.spin_ros)
        cls.spin_thread.daemon = True
        cls.spin_thread.start()
        
        # testing request constant urls
        cls.BASE_URL = "http://127.0.0.1:8080"
        cls.WS_PUB_URL = "ws://127.0.0.1:8080/ws_publish"
        
        # wait for ros2 node to be initialized
        time.sleep(2)

    @classmethod
    def spin_ros(cls):
        rclpy.spin(cls.test_node)

    def setUp(self):
        # Clear received messages array before each test to avoid interference
        self.test_node.received_messages.clear()

    async def websocket_connect(self):
        websocket = await websockets.connect(self.WS_PUB_URL)
        await asyncio.sleep(0.5)
        return websocket

    async def send_websocket_message(self, topic, message):
        """Send a message to the WebSocket server"""
        async with websockets.connect(self.WS_PUB_URL) as websocket:
            data = {
                "topic": topic,
                "message": message
            }
            await websocket.send(json.dumps(data))
            await asyncio.sleep(1)

    def test_http_endpoint(self):
        """Test the basic HTTP endpoint"""
        response = requests.get(f"{self.BASE_URL}/admin/test")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"message": "Hello World"})

    def test_websocket_publisher(self):
        """Test publishing messages via WebSocket"""
        async def run_test():
            test_topic = "/test_websocket_pub"
            self.test_node.create_test_subscription(test_topic)
            
            await asyncio.sleep(1)
            
            test_message = "Hello, my name is Karan"
            await self.send_websocket_message(test_topic, test_message)
            
            await asyncio.sleep(2)
            
            self.assertIn(test_topic, self.test_node.received_messages)
            self.assertIn(test_message, self.test_node.received_messages[test_topic])

        asyncio.run(run_test())

    def test_websocket_subscriber(self):
        """Test receiving messages via WebSocket"""
        async def run_test():
            test_topic = "/test_websocket_sub"
            test_message = "Test message pls work"

            # Creates new HTTP session to modify node's subscribed_topics
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.BASE_URL}/admin/add_subscription", 
                    json={"topic": test_topic}
                ) as response:
                    self.assertEqual(response.status, 200)

            await asyncio.sleep(2)

            # Connect to ws 
            ws = await self.websocket_connect()
            
            try:
                # Create publisher and publish 5 messages
                publisher = self.test_node.create_publisher(String, test_topic, 10)
                await asyncio.sleep(1)
                for i in range(5):
                    msg = String()
                    msg.data = f"{test_message}_{i}"
                    publisher.publish(msg)
                    await asyncio.sleep(0.5)                

                messages_received = []
                start_time = time.time()
                
                while time.time() - start_time < 10:  # Trying to recive for 10s, this can be adjusted if needed 
                    try:
                        result = await asyncio.wait_for(ws.recv(), timeout=1)
                        messages_received.append(json.loads(result))
                    except asyncio.TimeoutError:
                        # if even one message is received, break the loop
                        if messages_received:  
                            break
                        await asyncio.sleep(0.1)
                        continue
                    
                self.assertTrue(len(messages_received) > 0, "No messages received")
                
                # check to see if the message is in the received messages
                self.assertTrue(
                    any(
                        msg["topic"] == test_topic and test_message in msg["message"]
                        for msg in messages_received
                    ),
                    "No matching messages found"
                )
                
                publisher.destroy()
                
            finally:
                await ws.close()

        asyncio.run(run_test())

    def test_concurrent_connections(self):
        """Test multiple concurrent WebSocket connections"""
        async def run_concurrent_test():
            ws_connections = []
            test_topics = []
            
            for i in range(3):
                topic = f"/concurrent_test_{i}"
                test_topics.append(topic)
                self.test_node.create_test_subscription(topic)
            
            await asyncio.sleep(1)
            
            for i, topic in enumerate(test_topics):
                ws = await self.websocket_connect()
                ws_connections.append(ws)
                await self.send_websocket_message(topic, f"Concurrent message {i}")
            
            await asyncio.sleep(2)
            
            for i, topic in enumerate(test_topics):
                self.assertIn(topic, self.test_node.received_messages)
                self.assertIn(f"Concurrent message {i}", 
                            self.test_node.received_messages[topic])
            
            for ws in ws_connections:
                await ws.close()

        asyncio.run(run_concurrent_test())

    @classmethod
    def tearDownClass(cls):
        cls.test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()