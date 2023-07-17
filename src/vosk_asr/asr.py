import json
import queue
import rospy
import sounddevice as sd

from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

class VoskSpeechRecognitionResponder:
    def __init__(self, stream, model):
        self.stream = stream
        device_info = sd.query_devices(None, 'input') # queries default sound device
        # soundfile expects an int, sounddevice provides a float:
        self.sample_rate = int(device_info['default_samplerate'])

        if not self._setDictionaryInternal(model, None):
            raise Exception('Unable to load dictionary')

    def _setDictionaryInternal(self, dictionary, language):
        try:
            if dictionary is not None and str(dictionary):
                print('Setting dictionary to %s' % dictionary)
                self.model = Model(model_name='vosk-model-' + dictionary)
            elif language is not None and str(language):
                print('Setting language to %s' % language)
                self.model = Model(lang=language)
            else:
                print('No dictionary or language specified')
                return False
        except SystemExit:
            print('Dictionary or language not available')
            return False

        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        return True

    def transcribe(self, frame):
        if self.rec.AcceptWaveform(frame):
            return (False, json.loads(self.rec.Result())['text'])
        else:
            return (True, json.loads(self.rec.PartialResult())['partial'])

    def muteMicrophone(self):
        print('Muting microphone')
        self.stream.abort()
        return True

    def unmuteMicrophone(self):
        print('Unmuting microphone')
        self.stream.start()
        return True

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def main():
    rospy.init_node('vosk_asr', anonymous=False, disable_signals=True)

    pub = rospy.Publisher('transcription', String, queue_size=10)
    model = rospy.get_param('~model', 'small-es-0.42')

    q = queue.Queue()

    with sd.RawInputStream(blocksize=8000,
                            dtype='int16',
                            channels=1,
                            callback=lambda indata, frames, time, status: q.put(bytes(indata))) as stream:
        responder = VoskSpeechRecognitionResponder(stream, model)

        while True:
            frame = q.get()
            isPartial, transcription = responder.transcribe(frame)

            if transcription:
                if not isPartial:
                    rospy.loginfo('result: %s' % transcription)
                    pub.publish(transcription)
                else:
                    rospy.loginfo('partial: %s' % transcription)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

