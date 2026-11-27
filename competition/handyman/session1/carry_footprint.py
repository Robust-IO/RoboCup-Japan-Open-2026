"""Scoped local/global costmap footprint parameters, restored before child exit."""
import json
class FootprintLease:
    def __init__(self,node,wait,polygon):self.node=node;self.wait=wait;self.polygon=polygon;self.saved=[]
    def apply(self):
        from rcl_interfaces.srv import GetParameters,SetParametersAtomically
        from rclpy.parameter import Parameter
        for name in ('/local_costmap/local_costmap','/global_costmap/global_costmap'):
            get=self.node.create_client(GetParameters,name+'/get_parameters')
            put=self.node.create_client(SetParametersAtomically,name+'/set_parameters_atomically')
            if not get.wait_for_service(timeout_sec=5) or not put.wait_for_service(timeout_sec=5):raise ValueError('carry_costmap_service_missing:'+name)
            req=GetParameters.Request();req.names=['footprint'];old=self.wait(get.call_async(req)).values[0]
            if old.type!=4:raise ValueError('carry_footprint_parameter_not_string:'+name)
            # Preserve exact parameter type and value, including empty footprint string.
            restore=Parameter('footprint',value=old.string_value).to_parameter_msg()
            self.saved.append((put,restore))
            req=SetParametersAtomically.Request();req.parameters=[Parameter('footprint',value=json.dumps(self.polygon)).to_parameter_msg()]
            result=self.wait(put.call_async(req)).result
            if not result.successful:raise ValueError('carry_footprint_rejected:'+result.reason)
            req=GetParameters.Request();req.names=['footprint']
            if json.loads(self.wait(get.call_async(req)).values[0].string_value)!=self.polygon:raise ValueError('carry_footprint_readback_failed')
    def restore(self):
        from rcl_interfaces.srv import SetParametersAtomically
        errors=[]
        for put,value in reversed(self.saved):
            try:
                req=SetParametersAtomically.Request();req.parameters=[value]
                result=self.wait(put.call_async(req)).result
                if not result.successful:raise ValueError(result.reason)
            except Exception as exc:errors.append(str(exc))
        if errors:raise ValueError('carry_footprint_restore_failed:'+repr(errors))
