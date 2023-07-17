import json
import queue
import rospy
import sounddevice as sd

from std_msgs.msg import String
from std_srvs.srv import SetBool

from vosk import Model, KaldiRecognizer

DEFAULT_MODEL = 'small-es-0.42'

class VoskSpeechRecognitionResponder:
    def __init__(self, stream):
        self.stream = stream

        device_info = sd.query_devices(None, 'input') # queries default sound device
        # soundfile expects an int, sounddevice provides a float:
        self.sample_rate = int(device_info['default_samplerate'])

        model = rospy.get_param('~model', DEFAULT_MODEL)

        try:
            rospy.loginfo('setting dictionary to %s' % model)
            self.model = Model(model_name='vosk-model-' + model)
        except SystemExit:
            rospy.logfatal('dictionary or language not available')

        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        self.pub = rospy.Publisher('echo_transcription', String, queue_size=10)
        self.srv = rospy.Service('mute_microphone', SetBool, self._handle_microphone)

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

    def _handle_microphone(self, req):
        if req.data:
            rospy.loginfo('muting microphone')
            self.stream.abort()
            ret = 'muted'
        else:
            rospy.loginfo('unmuting microphone')
            self.stream.start()
            ret = 'unmuted'

        return (True, ret)

def main():
    rospy.init_node('asr', anonymous=False, disable_signals=True)

    q = queue.Queue()

    with sd.RawInputStream(blocksize=8000,
                            dtype='int16',
                            channels=1,
                            callback=lambda indata, frames, time, status: q.put(bytes(indata))) as stream:
        responder = VoskSpeechRecognitionResponder(stream)

        while True:
            responder.transcribe(q.get())

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

