"""Write measured diagnostic captures without delaying an action's age check.

Source captures consumed by inference remain synchronous. Shutdown drains all
writes and propagates disk failures. Arrays belong to a single immutable sample.
"""
from concurrent.futures import ThreadPoolExecutor

class DeferredMeasurementCaptures:
    def save(self,sample,name,measurement=None):
        if measurement is None:return super().save(sample,name,measurement)
        previous=getattr(self,'last_saved_observation',None)
        if previous is not None and previous[0] is sample and previous[1] is measurement:return previous[2]
        if not hasattr(self,'capture_queue'):
            self.capture_queue=ThreadPoolExecutor(max_workers=1);self.capture_futures=[]
        from canned_grasp_runtime import CanRuntime
        path=self.root/'captures'/name
        def write():
            result=CanRuntime.save(self,sample,name,measurement)
            (result/'can-measurement.json').rename(result/'object-measurement.json')
            return result
        self.capture_futures.append(self.capture_queue.submit(write))
        self.last_saved_observation=(sample,measurement,path)
        return path

    def shutdown(self):
        try:
            if hasattr(self,'capture_queue'):
                self.capture_queue.shutdown(wait=True)
                for future in self.capture_futures:future.result()
        finally:super().shutdown()
