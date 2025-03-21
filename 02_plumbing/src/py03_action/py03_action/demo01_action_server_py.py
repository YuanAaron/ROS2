'''
处理提交的目标值和处理客户端发送的取消请求 都已经有默认实现(前者默认接受，后者默认拒绝），
如果默认实现不符合需求，也可以重写这两个回调函数。
'''
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from base_interfaces.action import Progress

class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progress_action_server_node_py")
        self._action_server = ActionServer(self, Progress, "action_sum", 
                                           execute_callback=self.execute_callback, 
                                           goal_callback=self.goal_callback, 
                                           cancel_callback=self.cancel_callback)
        self.get_logger().info("动作服务端创建，等待请求...")

    #重新处理提交的目标值的回调函数
    def goal_callback(self, goal_request):
        self.get_logger().info("接收到动作客户端请求，请求数字为：%ld" %goal_request.num)
        if goal_request.num < 1:
            self.get_logger().error("提交的目标值必须大于等于1")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        #当前是无条件同意取消，但在真实场景中需要判断是否允许取消请求
        self.get_logger().info("接收到任务取消请求")
        return CancelResponse.ACCEPT


    #生成连续反馈和响应最终结果
    #goal_handle是ServerGoalHandle类型
    #goal_handle.request可以这样做是因为@property（装饰器）把request方法变成了一个属性，这样就可以像访问属性一样去调用这个方法，而无需像调用函数那样使用括号。
    def execute_callback(self, goal_handle):
        self.get_logger().info("开始执行任务...")

        #生成连续反馈，返回给客户端
        feedback = Progress.Feedback()
        result = Progress.Result()
        sum = 0
        for i in range(1, goal_handle.request.num + 1):
            sum += i

            #检查是否有取消请求
            #TODO 取消请求尝试多次仍然无效
            if goal_handle.is_cancel_requested:
                result.sum = sum
                goal_handle.canceled()
                self.get_logger().info("任务取消!")
                return result

            feedback.progress = i / goal_handle.request.num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("连续反馈中，进度为：%.2f" %feedback.progress)
            time.sleep(1)

        #响应最终结果
        result.sum = sum
        goal_handle.succeed() #设置结果状态
        self.get_logger().info("任务完成!")
        return result #TODO 怎么知道要返回呢

def main():
    rclpy.init()
    node = ProgressActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
