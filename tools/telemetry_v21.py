"""V21 UART binary decoder, CSV export and measured-event statistics (stdlib)."""
from pathlib import Path
import argparse
import binascii
import csv
import json
import math
import struct

MAGIC = b'SV21'
U16 = {8:'camera_frame',10:'display_frame',12:'cx',14:'cy',16:'pan_us',18:'tilt_us',
       20:'ae_register',22:'gain_register',28:'led_sample_id',30:'led_timeouts',
       32:'rx_errors',34:'bad_frames',36:'input_overflow',38:'output_overflow',
       40:'output_underflow',42:'tracker_overrun',44:'ae_dropped',46:'record_dropped',
       48:'telemetry_dropped',54:'decision_cycles',56:'pwm_frame',64:'measurement_epoch',
       66:'x0',68:'y0',70:'x1',72:'y1'}
U32 = {24:'led_cycles',50:'uptime_ms',58:'pwm_wait_cycles'}
FLAGS = ['measurement','observed','ambiguous','target_ae','motion_exposure_cap',
         'overlay_matched','at_limit','stale']
COUNTERS = ['led_timeouts','rx_errors','bad_frames','input_overflow','output_overflow',
            'output_underflow','tracker_overrun','ae_dropped','record_dropped','telemetry_dropped']

class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0
        self.crc_errors = 0
        self.unsupported = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        records = []
        while True:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                # Keep a possible partial header across arbitrary read chunks.
                keep = min(3, len(self.buffer))
                self.discarded += len(self.buffer)-keep
                if keep:
                    self.buffer[:] = self.buffer[-keep:]
                else:
                    self.buffer.clear()
                break
            if pos:
                self.discarded += pos
                del self.buffer[:pos]
            if len(self.buffer)<80:
                break
            packet = bytes(self.buffer[:80])
            if binascii.crc_hqx(packet[:78],0xffff) != int.from_bytes(packet[78:],'big'):
                self.crc_errors += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            del self.buffer[:80]
            if packet[4]!=1 or packet[63]!=1:
                self.unsupported += 1
                continue
            row = {name:struct.unpack_from('>H',packet,offset)[0] for offset,name in U16.items()}
            row.update({name:struct.unpack_from('>I',packet,offset)[0] for offset,name in U32.items()})
            row.update({name:bool(packet[5]&(1<<i)) for i,name in enumerate(FLAGS)})
            row.update(version=packet[4],quality=packet[6],status=packet[7],led_valid=bool(packet[62]))
            records.append(row)
        return records

def distribution(values):
    values = sorted(values)
    if not values:
        return {'n':0,'min':None,'p50':None,'p95':None,'max':None}
    def percentile(p):
        return values[max(0,math.ceil(p*len(values))-1)]
    return {'n':len(values),'min':values[0],'p50':percentile(.5),'p95':percentile(.95),'max':values[-1]}

def enrich(rows, pixel_hz=75000000, camera_hz=60000000, width=1280, height=720):
    if pixel_hz<=0 or camera_hz<=0 or width<=0 or height<=0:
        raise ValueError('Clocks and image dimensions must be positive')
    session = 0
    previous_time = None
    for original in rows:
        row = dict(original)
        if previous_time is not None and row['uptime_ms']<previous_time:
            session += 1   # Reboot or 32-bit uptime wrap; do not merge event IDs.
        previous_time = row['uptime_ms']
        row.update(session=session,
                   decision_ms=row['decision_cycles']*1000/camera_hz,
                   pwm_wait_ms=row['pwm_wait_cycles']*1000/pixel_hz,
                   led_response_ms=row['led_cycles']*1000/pixel_hz,
                   estimated_centre_error_x=row['cx']-width/2,
                   estimated_centre_error_y=row['cy']-height/2)
        yield row

def summarize(rows, pixel_hz=75000000, camera_hz=60000000, width=1280, height=720):
    led = []
    pwm = []
    decision = []
    centre_x = []
    centre_y = []
    last_led = last_pwm = last_frame = None
    observed = ambiguous = snapshots = 0
    segments = {}
    for row in rows:
        snapshots += 1
        reliable = row['observed'] and row['quality']==2 and not row['stale']
        observed += bool(reliable)
        ambiguous += row['ambiguous']
        session = row['session']
        segment = segments.setdefault(session,{'first_uptime_ms':row['uptime_ms'],'last_uptime_ms':row['uptime_ms'],
                                              'counters':{name:{'first':row[name]} for name in COUNTERS}})
        segment['last_uptime_ms'] = row['uptime_ms']
        for name in COUNTERS:
            item = segment['counters'][name]
            item.update(last=row[name],delta_since_first_snapshot=row[name]-item['first'],
                        saturated=row[name]==65535)
        led_key = (session,row['measurement_epoch'],row['led_sample_id'])
        if row['measurement'] and row['led_valid'] and row['led_cycles']>0 and led_key!=last_led:
            led.append(row['led_response_ms'])
            last_led = led_key
        pwm_key = (session,row['pwm_frame'],row['pwm_wait_cycles'])
        if row['pwm_wait_cycles']>0 and pwm_key!=last_pwm:
            pwm.append(row['pwm_wait_ms'])
            last_pwm = pwm_key
        frame_key = (session,row['camera_frame'])
        if frame_key!=last_frame:
            if row['decision_cycles']>0:
                decision.append(row['decision_ms'])
            if reliable:
                centre_x.append(abs(row['estimated_centre_error_x']))
                centre_y.append(abs(row['estimated_centre_error_y']))
            last_frame = frame_key
    return {'snapshots':snapshots,'sessions':segments,
            'reliable_observed_snapshot_fraction':observed/snapshots if snapshots else None,
            'ambiguous_snapshots':ambiguous,
            'led_command_to_internal_video_detection_ms':distribution(led),
            'accepted_record_to_pwm_latch_ms':distribution(pwm),
            'camera_postframe_decision_ms':distribution(decision),
            'estimated_abs_centre_error_x_px':distribution(centre_x),
            'estimated_abs_centre_error_y_px':distribution(centre_y),
            'conversion':{'pixel_hz':pixel_hz,'camera_hz':camera_hz,'width':width,'height':height},
            'percentile_method':'nearest rank',
            'scope':[
                'UART snapshots can miss events; counts describe received coverage only.',
                'LED response includes camera/display pipeline to internal detection, not screen photons.',
                'PWM wait starts at an accepted observation, not target physical motion or mechanical settling.',
                'Estimated centroid error is not independently annotated localization accuracy.',
                'AE/gain fields are raw registers, not calibrated exposure milliseconds.',
                'Clock conversions are nominal and must match the actual build.',
                'Session boundary means uptime decreased: reboot or uptime wrap.']}

def analyze(path, prefix, **settings):
    decoder = Decoder()
    rows = []
    with Path(path).open('rb') as stream:
        while True:
            chunk = stream.read(65536)
            if not chunk:
                break
            rows.extend(decoder.feed(chunk))
    rows = list(enrich(rows,**settings))
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True,exist_ok=True)
    with prefix.with_suffix('.csv').open('w',newline='',encoding='utf-8-sig') as output:
        if rows:
            writer = csv.DictWriter(output,fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    summary = summarize(rows,**settings)
    summary['input'] = str(Path(path).resolve())
    summary['decode'] = {'crc_errors':decoder.crc_errors,'unsupported_packets':decoder.unsupported,
                         'discarded_bytes':decoder.discarded,'trailing_bytes':len(decoder.buffer)}
    prefix.with_suffix('.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return summary

if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--pixel-hz',type=float,default=75000000)
    parser.add_argument('--camera-hz',type=float,default=60000000)
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=720)
    args = parser.parse_args()
    result = analyze(args.raw,args.out,pixel_hz=args.pixel_hz,camera_hz=args.camera_hz,width=args.width,height=args.height)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['snapshots']:
        raise SystemExit('No supported CRC-valid V21 packets decoded')
