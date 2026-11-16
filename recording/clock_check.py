#!/usr/bin/env python3
"""Read-only wall-clock sanity/stability check. Never changes time or services."""
import argparse
import datetime
import json
import os
import subprocess
import time
import xmlrpc.client


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def check(seconds=3., limit=.05):
    if seconds <= 0 or limit <= 0:
        raise ValueError('Invalid clock observation limits')
    start_mono, start_wall = time.monotonic(), time.time()
    instant = datetime.datetime.fromtimestamp(start_wall,datetime.timezone.utc)
    result = dict(state='passed', utc=instant.isoformat(),
                  observed_seconds=seconds, max_wall_monotonic_delta_s=0., faults=[], warnings=[])
    # A reset RTC must be corrected before sensor/EKF startup, never during recording.
    if instant.year < 2024:
        result['faults'].append('system_clock_before_2024: synchronize before starting sensors/EKF')
    try:
        master=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'),transport=TimeoutTransport())
        code, unused, simulated=master.getParam('/bagtools_clock_check','/use_sim_time')
        if code==1 and simulated is True:result['faults'].append('use_sim_time_true_on_live_recorder')
    except (OSError,xmlrpc.client.Error):
        result['warnings'].append('ros_clock_mode_unavailable_before_bringup')
    while time.monotonic()-start_mono < seconds:
        time.sleep(.1)
        now_mono, now_wall = time.monotonic(), time.time()
        delta = abs((now_wall-start_wall)-(now_mono-start_mono))
        result['max_wall_monotonic_delta_s'] = max(result['max_wall_monotonic_delta_s'], delta)
        if delta > limit:
            result['faults'].append('system_clock_unstable'); break
    try:
        out = subprocess.check_output(['chronyc','tracking'], stderr=subprocess.STDOUT, timeout=2).decode('utf-8','replace')
        result['chrony_tracking'] = out
        if 'Not synchronised' in out:
            result['warnings'].append('chrony_not_synchronised; observed stability is not a future guarantee')
    except (OSError,subprocess.SubprocessError):
        result['warnings'].append('chrony_tracking_unavailable')
    result['state'] = 'failed' if result['faults'] else 'passed'
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=3.)
    ap.add_argument('--output');a=ap.parse_args(); result=check(a.seconds)
    print(json.dumps(result))
    if a.output:
        with open(a.output,'w') as f:json.dump(result,f,indent=2)
    return 0 if result['state']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
