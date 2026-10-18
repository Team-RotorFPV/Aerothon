"""Record the real certificate's predictions without changing its decisions."""
import inspect,json,textwrap
from pathlib import Path
from types import MethodType

class MotionForecastTrace:
    def __init__(self,node,path):
        self.node,self.path,self.records=node,Path(path),[]
        original=node._publish
        code=textwrap.dedent(inspect.getsource(original.__func__))
        # Match exact indentation from the frozen source rather than changing
        # numerical integration, timestamps, controls or certificate outcomes.
        line=next(line for line in code.splitlines() if 'packet_wait = capture_age+' in line)
        indent=line[:len(line)-len(line.lstrip())]
        assert code.count(line)==1
        code=code.replace(line,line+'\n'+indent+"prediction=dict(forecast_now=forecast_now,sample_epoch=sample_epoch,packet_wait=packet_wait,command=list(command),points_body=[],clear=None)\n"+
            indent+'self._prediction_attempts.append(prediction)')
        line=next(line for line in code.splitlines() if 'nxt = point+velocity*dt' in line)
        indent=line[:len(line)-len(line.lstrip())]
        code=code.replace(line,line+'\n'+indent+"prediction['points_body'].append([float(t),float(nxt[0]),float(nxt[1])])")
        line=next(line for line in code.splitlines() if 'if not self._planner.segment_clear(point,nxt):' in line)
        indent=line[:len(line)-len(line.lstrip())]
        code=code.replace(line,line+'\n'+indent+"    prediction['clear']=False")
        # Only the inner certificate has this return at this indentation.
        line=indent[:-4]+'return True'
        assert code.count(line)==1
        code=code.replace(line,indent[:-4]+"prediction['clear']=True\n"+line)
        namespace=dict(original.__func__.__globals__)
        exec(compile(code,'<recorded original motion certificate>','exec'),namespace)
        instrumented=MethodType(namespace['_publish'],node)
        def capture(_node,*args,**kwargs):
            _node._prediction_attempts=[]
            before=dict(clock_s=_node._now(),pose_t=_node._pose_t,velocity_t=_node._fcu_velocity_t,
                pose=_node._pos,yaw=_node._yaw,ground_world=_node._ground_world.tolist(),
                wind_world=_node._wind_world.tolist(),wind_rate_world=_node._wind_rate_world.tolist(),scan_t=_node._scan_t,scan_pose=_node._scan_pose,scan_pairing_reason=getattr(_node,'_scan_pairing_reason',None),pending_scan_stamp=(_node._pending_paired_scan[0] if getattr(_node,'_pending_paired_scan',None) else None))
            result=instrumented(*args,**kwargs)
            self.records.append(dict(before=before,after_clock_s=_node._now(),
                command_history_last=(_node._command_history[-1][0] if _node._command_history else None),
                predictions=_node._prediction_attempts,detail=_node.last_detail))
            return result
        node._publish=MethodType(capture,node)
    def close(self):
        with self.path.open('x') as h:json.dump(dict(
            scope='Onboard pose, velocity, issued-command history and original certificate predictions only',
            controller_decisions_changed=False,simulator_truth_input=False,records=self.records),h)
