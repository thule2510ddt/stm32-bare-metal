## Audio Record and Playback on STM32F746 Discovery 📝  
Using STM32F746NG discovery board to record audio by using internal MEMS digital microphone, save it into micro SD card in .wav form and play it back trough the audio line out jack.

### Harware Diagram
<img src="https://github.com/tzf230201/Record_Playback-32F746GDISCOVERY/blob/main/picture/hardware%20diagram.jpg" height="240" width="680" >

### Record
BTW if the SD card doesn't plugged in, it will record for ever
<img src="https://github.com/tzf230201/Record_Playback-32F746GDISCOVERY/blob/main/picture/recording.jpg" height="450" width="680" >

### Playback
<img src="https://github.com/tzf230201/Record_Playback-32F746GDISCOVERY/blob/main/picture/playing.jpg" height="450" width="680" >

### Saved file
<img src="https://github.com/tzf230201/Record_Playback-32F746GDISCOVERY/blob/main/picture/wav%20file.png" height="200" width="680" >


#### *Note
- This code was built on STM32CubeIDE 1.10.0 🏠
- Easy Use. This repo can be directly imported as a STM32CubeIDE project
- You can easily getting started by modify the code in Core/Src/role.c file 🚀
- Change record duration in Core/Inc/record.h -> DEFAULT_TIME_REC

# stm32-bare-metal
