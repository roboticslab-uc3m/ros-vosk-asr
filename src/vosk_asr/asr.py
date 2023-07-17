import json
import queue
import rospy
import sounddevice as sd

from std_msgs.msg import String
from std_srvs.srv import Trigger

from vosk import Model, KaldiRecognizer

# inspired by https://github.com/alphacep/vosk-api/blob/12f29a3/python/example/test_microphone.py

DEFAULT_MODEL = 'small-es-0.42'

class VoskSpeechRecognitionResponder:
    def __init__(self, device, stream):
        self.stream = stream

        device_info = sd.query_devices(device, 'input') # queries default sound device
        # soundfile expects an int, sounddevice provides a float:
        self.sample_rate = int(device_info['default_samplerate'])

        model = rospy.get_param('~model', DEFAULT_MODEL)

        try:
            rospy.loginfo('setting dictionary to %s' % model)
            self.model = Model(model_name='vosk-model-' + model)
        except SystemExit as e:
            rospy.logfatal('dictionary or language not available')
            raise e

        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        self.pub = rospy.Publisher('transcription', String, queue_size=10)
        self.srv_mute = rospy.Service('mute_microphone', Trigger, self._mute_microphone)
        self.srv_unmute = rospy.Service('unmute_microphone', Trigger, self._unmute_microphone)

    def transcribe(self, frame):
        if self.rec.AcceptWaveform(frame):
            is_partial, transcription = False, json.loads(self.rec.Result())['text']
        else:
            is_partial, transcription = True, json.loads(self.rec.PartialResult())['partial']

        if transcription:
            if not is_partial:
                rospy.loginfo('result: %s' % transcription)
                self.pub.publish(transcription)
            else:
                rospy.loginfo('partial: %s' % transcription)

    def _mute_microphone(self, req):
        rospy.loginfo('muting microphone')
        self.stream.abort()
        return (True, 'muted')

    def _unmute_microphone(self, req):
        rospy.loginfo('unmuting microphone')
        self.stream.start()
        return (True, 'muted')

def main():
    rospy.init_node('asr', anonymous=False, disable_signals=True)

    q = queue.Queue()
    device = rospy.get_param('~device', None)

    with sd.RawInputStream(blocksize=8000,
                           device=device,
                           dtype='int16',
                           channels=1,
                           callback=lambda indata, frames, time, status: q.put(bytes(indata))) as stream:
        responder = VoskSpeechRecognitionResponder(device, stream)

        while True:
            responder.transcribe(q.get())

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

