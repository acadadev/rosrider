#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import asyncio

class KeepoutFilterLauncher(Node):
    def __init__(self):
        super().__init__('keepout_filter_launcher')

    async def call_service_async(self, service_name, service_type, request):
        self.get_logger().info(f'Calling service: {service_name}')
        client = self.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {service_name} not available')
            return None
        future = client.call_async(request)
        try:
            # Use rclpy.spin_until_future_complete for ROS2 async operations
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.done():
                response = future.result()
                if response is not None:
                    self.get_logger().info(f'Service call successful: {response.success}, message: {response.message}')
                    return response
                else:
                    self.get_logger().error(f'Service call failed with exception: {future.exception()}')
                    return None
            else:
                self.get_logger().error(f'Service call timed out')
                return None
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
            return None

    async def toggle_filter_sequence(self):

        global_enable_request = SetBool.Request(data=True)
        global_enable_result = await self.call_service_async('/global_costmap/keepout_filter/toggle_filter', SetBool, global_enable_request)

        await asyncio.sleep(2.0)

        if global_enable_result and global_enable_result.success:
            self.get_logger().info(f'Global Keepout Filter enabled: {global_enable_result.message}')
            await asyncio.sleep(2.0)
        else:
            self.get_logger().error('Failed to enable Global Keepout Filter')
            return False

        local_enable_request = SetBool.Request(data=True)
        local_enable_result = await self.call_service_async('/local_costmap/keepout_filter/toggle_filter', SetBool, local_enable_request)

        if local_enable_result and local_enable_result.success:
            self.get_logger().info(f'Local Keepout Filter enabled: {local_enable_result.message}')
            return True
        else:
            self.get_logger().error('Failed to enable Local Keepout Filter')
            return False


def main():
    rclpy.init()
    launcher = KeepoutFilterLauncher()
    try:
        success = asyncio.run(launcher.toggle_filter_sequence())
        if success:
            launcher.get_logger().info("Keepout Filter enable sequence completed successfully")
        else:
            launcher.get_logger().error("Keepout Filter enable sequence failed")
    except Exception as e:
        launcher.get_logger().error(f'Main function error: {e}')
        import traceback
        launcher.get_logger().error(f'Stack trace: {traceback.format_exc()}')
    finally:
        launcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
