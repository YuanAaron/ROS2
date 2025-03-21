import sys
import rclpy
from rclpy.node import Node
#from rclpy.action.client import ClientGoalHandle
from rclpy.logging import get_logger
from rclpy.action import ActionClient

from base_interfaces.action import Progress

class ProgressActionClient(Node):
    def __init__(self):
        super().__init__("progress_action_client_node_py")
        self._action_client = ActionClient(self, Progress, "action_sum");
    
    #发送目标请求
    def send_goal(self, num):
        #连接动作服务端
        self._action_client.wait_for_server(10)

        self.get_logger().info("发送请求数据！")
        goal =  Progress.Goal()
        goal.num = num
        self._send_goal_future = self._action_client.send_goal_async(goal, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    #处理目标发送后的响应
    def goal_response_callback(self, future): #TODO 怎么知道参数是future呢？？？
        goal_handle= future.result()
        #怎么知道future.result()的返回值是什么呢？
        #方法一：从输出的结果<class 'rclpy.action.client.ClientGoalHandle'>去查看其源码，就可以知道该类有哪些属性和方法
        #方法二：将输出结果<class 'rclpy.action.client.ClientGoalHandle'>导入进来，然后显式声明goal_handle的类型，
        #       即goal_handle:ClientGoalHandle = future.result()，这样后面使用goal_handle时就有属性和方法的提示了
        self.get_logger().info(str(type(goal_handle))) #或 goal_handle.__str__()，后者不如前者

        if not goal_handle.accepted:
            self.get_logger().error("目标请求被服务器拒绝！")
            return
            
        self.get_logger().info("目标被接收，等待结果中...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)


    #处理连续反馈
    def feedback_callback(self, feedback): #TODO 怎么知道参数是feedback？？？怎么知道feedback是什么呢？？？
        progress = feedback.feedback.progress * 100
        self.get_logger().info("当前进度：%d%%" %progress)

    
    #处理最终响应
    def result_callback(self, future):
        result = future.result().result;
        self.get_logger().info("任务执行完毕，最终结果为：%ld" %result.sum)
        rclpy.shutdown()

def main():
    if len(sys.argv) != 2:
        get_logger("rclpy").error("请提交一个整数！")
        return

    rclpy.init()
    node = ProgressActionClient()
    node.send_goal(int(sys.argv[1]))
    rclpy.spin(node)
    #rclpy.shutdown()

if __name__ == '__main__':
    main()