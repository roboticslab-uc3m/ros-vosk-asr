# ros-vosk-asr
An ASR service that runs the Vosk/Kaldi engine. Tested on ROS Noetic (Ubuntu 20.04).

## Dependencies

`pip install vosk sounddevice`

## Usage

Some sample invocations via `roslaunch`:
- `roslaunch vosk_asr asr model:=small-es-0.42` (default, also loaded via `rosrun vosk_asr asr`)
- `roslaunch vosk_asr asr model:=small-en-us-0.4`
- `roslaunch vosk_asr asr model:=es-0.42` (warning: 1.4 GB)
- `roslaunch vosk_asr asr model:=en-us-0.42-gigaspeech` (warning: 2.3 GB)

To list and download the desired models offline, you can use `vosk-transcriber --list-models`. Otherwise, the requested model will be downloaded on demand during node initialization.

See also: https://alphacephei.com/vosk/models.

## Communications

Topics:
- `/transcription` (`std_msgs/String` ): result of the transcription

Services:
- `/mute_microphone` (`std_srvs/Trigger`)
- `/unmute_microphone` (`std_srvs/Trigger`)
