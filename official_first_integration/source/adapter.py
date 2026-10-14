#!/usr/bin/env python3
"""ROS launch entry for source reconstruction; original binaries are never run."""
import argparse,importlib.util,signal,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
# Resolve source Runtime before loading the shared ROS integration adapter.
from runtime import Runtime
from task_adapter import adapter_class
spec=importlib.util.spec_from_file_location('nos_ros_transport',HERE.parent/'native/adapter.py')
transport=importlib.util.module_from_spec(spec);spec.loader.exec_module(transport)
def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--planner-mode',choices=('dwa','line'),default='dwa')
    args=parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]);rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('first_source_adapter');adapter=None;running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        adapter=adapter_class(transport.Adapter)(node,args.output,runtime_factory=lambda output:Runtime(output,args.planner_mode))
        while running and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if adapter:adapter.close()
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
