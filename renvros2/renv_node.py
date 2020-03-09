import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int32

import os, sys, traceback, logging, codecs, argparse, threading

from logging import getLogger

from renv_device import RenvDevice, actionHandler, event
import msg_parser.parser2 as parser2

host = "192.168.1.24:8080"

def log(*args, **kwargs):
    print(*args)

_type_dir = {
    'bool': 'Int',
    'int8': 'Int',
    'uint8': 'Int',
    'int16': 'Int',
    'uint16': 'Int',
    'int32': 'Int',
    'uint32': 'Int',
    'int64': 'Int',
    'uint64': 'Int',
    'float32': 'Double',
    'float64': 'Double',
    'boolean': 'Int',
    'float': 'Double',
    'double': 'Double',
    'string': 'String',
    'time': 'String',
}


def reverseConvert(data, paramInfo):
    if paramInfo is None:
        print('Invalid ParamInfo')
        return None

    if paramInfo['rosDataTypeName'] == 'time':
        return rclpy.time.Time()
    elif paramInfo['paramType'] == 'String':
        return str(data)
    elif paramInfo['paramType'] == 'Double':
        return float(data)
    return data

def convertTypeName(typeObj):
    typeName = typeObj.fullName
    if not typeObj.is_primitive:
        raise Exception('ROS Type can not convert(%s)' % typeObj.name)
    return _type_dir[typeName]


class ProxyRenvDevice(RenvDevice):
    """
    renvros2.ProxyRenvDevice
    """
    def __init__(self, typeId, name, version="1.0.0", device_uuid=None, deviceName=None, logger=None):
        """
        """

        RenvDevice.__init__(self, typeId, name, version=version, device_uuid=device_uuid, use_mta=False, deviceName=deviceName, logger=logger)
        self._msg_buffer = []
        pass

    def run(self):
        self.__run_thread = threading.Thread(target=lambda : self.run_forever())
        self.__run_thread.start()

    def stop(self):
        self.stop_running()
        self.__run_thread.join()


    def buildParamInfoFromROSType(self, type_obj, custom_comment=None):
        """
        :param type_obj:
        :return:
        """
        def parse_member(member, acc, prefix=''):
            # print('parsing member:', member)
            if member.is_constant: return # Do nothing if constant

            name = member.name if len(prefix) == 0 else prefix + '.' + member.name
            if not member.type.is_primitive:
                member.type.members.forEach(lambda child_member: parse_member(child_member, acc, name))
                return

            comment = name
            if custom_comment is not None and custom_comment.get(name, None):
                comment = custom_comment[name]

            info = self.buildParamInfo(name, convertTypeName(member.type), comment)
            info['rosDataTypeName'] = member.type.fullName
            acc.append(info)

        pis = []
        type_obj.members.forEach(lambda member: parse_member(member, pis))
        if len(pis) > 5:
            print('WARNING!! Renv2015 has the limitation of the number of parameter. Must be less than 6 (now %d)' % len(pis))
        return pis

    def buildParam(self, typeName, paramName, obj, feedback_type=False, result_type=False):
        # print('buildParam(', typeName, ',', paramName, ')')
        if paramName.find('.') >= 0:
            tokens = paramName.split('.')
            top_name = tokens[0]
            name = '.'.join(tokens[1:])
            if feedback_type:
                return self.buildParam(typeName, name, getattr(obj.feedback, top_name))
            elif result_type:
                return self.buildParam(typeName, name, getattr(obj.result, top_name))
            else:
                return self.buildParam(typeName, name, getattr(obj, top_name))
        else:
            if feedback_type:
                return self.translateData(typeName, getattr(obj.feedback, paramName))
            elif result_type:
                return self.translateData(typeName, getattr(obj.result, paramName))
            else:
                return self.translateData(typeName, getattr(obj, paramName))


    def translateData(self, typeName, value):
        if typeName == 'time':
            if value == 'now':
                return rospy.Time.now()
            return rospy.Time.now()
        elif typeName == 'duration':
            return rospy.Duration()
        # return value.sec + value.nsec / 1000.0 / 1000.0 / 1000.0
        elif typeName == 'bool':
            return 1 if value else 0
        elif typeName == 'string':
            return str(value)
        return value
            

    
class RenvNode(Node):

    

    def __init__(self,
                 node_name: str,
                 **kwargs
                 ):
        super().__init__(node_name, **kwargs)


        self.__proxy_renv_device = None
        self.__publishers = {}

        self.__subscribers = {}

        self.__clients = {}
        pass


    def init_renv(self, typeId, name, version="1.0.0", device_uuid=None, deviceName=None, logger=None):
        self.__proxy_renv_device = ProxyRenvDevice(typeId, name, version, device_uuid, deviceName, logger)
        self._logger = self.__proxy_renv_device.getLogger()
        pass

    def get_ros_publisher(self, topic_name):
        pub = self.__publishers.get(topic_name, None)
        #if pub is None:
        #    log('No publisher for %s is found' % topic_name)
        return pub

    def bind_param(self, obj, key, value, paramInfo):
        name = str(key) # unicode(key, 'utf-8').decode('utf-8')
        # print 'bind_param(', name, ',', value, ')'
        if name.find('.') >= 0:
            tokens = name.split('.')
            top_name = tokens[0]
            name = '.'.join(tokens[1:])
            if hasattr(obj, top_name):
                bind_param(getattr(obj, top_name), name, value, paramInfo)
        else:
            if hasattr(obj, name):
                # pis kara rosDataType de revesce convert suru
                # print('param:%s is %s/%s' % (name, value[u'val'], type(value[u'val'])))
                setattr(obj, name, reverseConvert(value[u'val'], paramInfo))
            else:
                print('Value(name=%s) is not included' % name)
    
    
    def create_renv_publisher(self, topic_type, topic_name, queue_size, *, callback=lambda x: x, topic_type_for_renv=None, custom_comment=None, custom_param_comment=None, without_ros_publisher=False, action_prefix='rostopic'):
        """

        :param topic_type: class object of topic. ex., Twist
        :param topic_name: default name of topic. ex., /cmd_vel
        :param queue_size: Queue size for topic message buffer.
        :param callback: Callback function called in message from renv is received.
        :param topic_type_for_renv: Topic type for Renv. This type will be noticed to Renv. Default is same type as topic_type parameter.
        :param custom_comment: Custome Comment for This topic
        :param custom_param_comment: Custom comment for parameter. This must be dictionary. key is parameter name, value is comment. ex., {"linear.x": "Velocity for X axis"}
        """
        parser = parser2.Parser2()

        topic_type_for_ros2 = topic_type
        if topic_type_for_renv is None: topic_type_for_renv = topic_type

        pis = []
        if topic_type_for_renv:
            rosStructInfo = parser.parse_msg_class2(topic_type_for_renv)
            pis = self.__proxy_renv_device.buildParamInfoFromROSType(rosStructInfo, custom_param_comment)
        
        
        comment = 'Renv-ROS Converter [Topic Subscriber="%s"]. ' % topic_name
        if topic_type_for_renv:
            comment += ' '.join(rosStructInfo.comment.split('\n'))
        if custom_comment: comment = custom_comment

        log('RenvNode.create_renv_publisher for topic named ', topic_name)

        def handler(msg, _self=self, without_ros_publiser=without_ros_publisher):
            # log('EventPublisher detect messages from renv:', topic_name)
            t = None
            if topic_type_for_renv:
                t = topic_type_for_renv()
            try:
                for key, value in msg.items():
                    ps = [p for p in pis if p['paramName'] == str(key)]
                    if len(ps) == 0:
                        log('WARNING: can not find ParamInfo(name=', str(key), ')')
                    if t: self.bind_param(t , key, value, ps[0])
            except:
                traceback.print_exc()

            pub = _self.get_ros_publisher(topic_name)
            if pub:
                pub.publish(callback(t))
            elif without_ros_publisher:
                try:
                    callback(t)
                except:
                    print('Exception in ActionHandler for Topic(topic_name=%s)' % (topic_name))
                    traceback.print_exc()
            pass

        self.__proxy_renv_device.addCustomPlainActionHandler(action_prefix + topic_name.replace('/','.'), comment, pis, handler)

        if without_ros_publisher:
            self.__publishers[topic_name] = None
            return None
        
        pub = self.create_publisher(topic_type_for_ros2, topic_name, queue_size)
        self.__publishers[topic_name] = pub
        return pub



    def create_renv_subscription(self, topic_type, topic_name, queue_size, *,
                                 callback=lambda x: x, topic_type_for_renv=None,
                                 custom_comment=None, custom_param_comment=None, event_prefix='rostopic',
                                 without_ros_subscription=False):
        """
        """
        parser = parser2.Parser2()

        topic_type_for_ros2 = topic_type
        if topic_type_for_renv is None: topic_type_for_renv = topic_type
        rosStructInfo = parser.parse_msg_class2(topic_type_for_renv)

        pis = self.__proxy_renv_device.buildParamInfoFromROSType(rosStructInfo, custom_param_comment)

        comment = 'Renv-ROS Converter [Topic Subscriber="%s"]. ' % topic_name 
        comment += ' '.join(rosStructInfo.comment.split('\n'))
        if custom_comment: comment = custom_comment
        
        func = self.__proxy_renv_device.addCustomEvent(event_prefix + topic_name.replace('/', '.'), comment, pis)
        
        def ros_callback(msg, en=topic_name, pis=pis, cb_=func):
            # print('ros_callback for msg: ', msg)
            v = {}
            for pi in pis:
                v[pi['paramName']] = self.__proxy_renv_device.buildParam(pi['rosDataTypeName'], pi['paramName'], msg, feedback_type=False, result_type=False)
            cb_(event_prefix + topic_name.replace('/', '.'), **v)
            pass

        if without_ros_subscription:
            return ros_callback
        
        sub = self.create_subscription(topic_type_for_ros2,
                                       topic_name,
                                       lambda x: ros_callback(callback(x)),
                                       queue_size)
        self.__subscribers[topic_name] = sub
        return sub

    def create_renv_client(self, service_type, service_name):
        cb_group = ReentrantCallbackGroup()
        service_type_for_ros2 = service_type

        result_sub = self.create_renv_subscription(service_type.Response,
                                                   service_name + '_response',
                                                   10,
                                                   without_ros_subscription=True,
                                                   event_prefix='rosservice')

        def request_callback(request_msg, self_=self, service_name=service_name):
            cli = self_.__clients[service_name]
            future = cli.call_async(request_msg)
            future.add_done_callback(lambda future: result_sub(future.result()))
            #rclpy.spin_until_future_complete(self_, future)

            #result = future.result()
            #result_sub(result)
            pass
        
        self.create_renv_publisher(service_type.Request,
                                   service_name + '_request',
                                   10,
                                   callback=request_callback,
                                   without_ros_publisher=True,
                                   action_prefix='rosservice')

        cli = self.create_client(service_type_for_ros2, service_name, callback_group=cb_group)
        self.__clients[service_name] = cli
        return cli

    def connect(self, host):
        """ Connect and run """
        self.__proxy_renv_device.connect(host)
        self.__proxy_renv_device.run()
        pass

    def disconnect(self):
        """ Stop and disconnect """
        self.__proxy_renv_device.stop()


    

class RenvActionClient():

    def __init__(self, node, action_type, action_name, action_type_for_renv=None, goal_converter=None, feedback_converter=None, result_converter=None):
        self.__action_client = ActionClient(node, action_type, action_name)

        if action_type_for_renv is None:
            action_type_for_renv = action_type
        if goal_converter is None:
            goal_converter = lambda x: x
        if feedback_converter is None:
            feedback_converter = lambda x: x
        if result_converter is None:
            result_converter = lambda x: x
            
        
        result_sub = node.create_renv_subscription(action_type_for_renv.Result,
                                                   action_name + '_result',
                                                   10,
                                                   without_ros_subscription=True,
                                                   event_prefix='rosaction')

#        goal_status_sub = node.create_renv_subscription(action_type_for_renv.Impl.GoalStatusMessage,
        goal_status_sub = node.create_renv_subscription(Int32,
                                                   action_name + '_goal_status',
                                                   10,
                                                   without_ros_subscription=True,
                                                   event_prefix='rosaction')

        
        def result_callback(future):
            result = future.result().result
            status = future.result().status
            #if status == GoalStatus.STATUS_SUCCEEDED:
            #    self.get_logger().info('Goal succeeded! Result: {0}'.format(result.result))
            #else:
            print('Goal with status: {0}'.format(status))
            # print(' Goal status type : ', type(status))
            # print(' dir(goal status) : ', dir(status))
            v = Int32()
            v.data = status
            goal_status_sub(v)
            result_sub(result_converter(result))
        
        send_goal_sub = node.create_renv_subscription(action_type_for_renv.Impl.SendGoalService.Response,
                                                   action_name + '_set_goal',
                                                   10,
                                                   without_ros_subscription=True,
                                                   event_prefix='rosaction')


        
        def goalset_done_callback(done_msg, send_goal_sub=send_goal_sub, self_=self):
            # print('Goal Set Done callback')
            self_._goal_handle = done_msg.result()
            if self_._goal_handle.accepted:
                #print ('Goal ACCEPTED!!!!')
                pass
            send_goal_sub(self_._goal_handle)

            self_._get_result_future = self_._goal_handle.get_result_async()
            self_._get_result_future.add_done_callback(result_callback)            
            pass
        
        feedback_sub = node.create_renv_subscription(action_type_for_renv.Feedback,
                                                     action_name + '_feedback',
                                                     10,
                                                     without_ros_subscription=True,
                                                     event_prefix='rosaction')
        
        def feedback_callback(feedback_msg, feedback_sub=feedback_sub, feedback_converter=feedback_converter):
            print('Receive Feedback {0}'.format(feedback_msg.feedback))
            feedback_sub(feedback_converter(feedback_msg.feedback))
            pass

        
        def goal_callback(goal_msg, feeback_callback=feedback_callback):
            #print('Receive Goal Msg from Renv.')
            #print(goal_msg)
            self._send_goal_future = self.__action_client.send_goal_async(goal_converter(goal_msg),
                                                    feedback_callback=feedback_callback)
            self._send_goal_future.add_done_callback(goalset_done_callback)
            pass

        node.create_renv_publisher(action_type_for_renv.Goal,
                                   action_name + '_goal',
                                   10,
                                   callback=goal_callback,
                                   without_ros_publisher=True,
                                   action_prefix='rosaction')

        cancel_sub = node.create_renv_subscription(action_type_for_renv.Impl.CancelGoalService.Response,
                                                   action_name + '_cancel_done',
                                                   10,
                                                   without_ros_subscription=True,
                                                   event_prefix='rosaction')
        
        def cancel_done_callback(future, cancel_sub=cancel_sub):
            print('Cancel Done msg from ROS2')
            cancel_response = future.result()
            cancel_sub(cancel_response)
            if len(cancel_response.goals_canceling) > 0:
                print('Goal successfully canceled')
            else:
                print('Goal failed to cancel')
            pass

        def cancel_callback(cancel_msg, self_=self):
            print('Receive Cancel Msg from Renv.')
            self_._send_cancel_future = self_._goal_handle.cancel_goal_async()
            self_._send_cancel_future.add_done_callback(cancel_done_callback)
            

        node.create_renv_publisher(None,
                                   action_name + '_cancel',
                                   10,
                                   callback=cancel_callback,
                                   without_ros_publisher=True,
                                   action_prefix='rosaction')
        

