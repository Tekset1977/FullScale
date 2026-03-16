# FullScale
Fullscale code and relevant development tidbits 

Different .inos are as follows: 
1.fullscalesleepinterrupt.ino is for sleep testing with altimeter interrupt
2.fullscaletest.ino is to test the stabilization 
3.flight_logger_p10.ino is for the code that is used for PDF

In the fullscaletest.ino is a file from 2/17/2026. I made the updates to be every 35ms. Which is not planned 20ms, but fairly close and I belive would yield usable data. Unfortunately, unlike ModifiedSub.ino it does not look beautiful, since there's a problem with Display and Sleep Mode. If those 2 things are gone, SD card write/flush works fine. I suspect its the I2C clutter and timing misfire, blocking the code from progression. Needs more testing. Debating wether or not to add failsafe to the code to make it run no matter what. Nasa would love it. Barometer was an issue, but I fixed it by writing directly to it and avoiding Adafruit. Otherwise, sample was too slow. Wondering how I got it to work the first time. Perhaps it wasn't even working the first time either and I was too naive and lacking scrutiny to notice. 

changelog since 2/18/2026. 
Modified fullscaletest.ino by adding verbose reset I2C bus and ICM. Display still doesn't work and there's a problem with the loop actually looping through. Tests show that the I2C bus is still taken up by something during new polling for data from MPL, which stalemates it. Maybe, if I do the aggregate: "send only sensor data every 25 loops to the display" it will fix itself. But there's still an issue of recording it to the SD file. I've been assured that the slower ICM should not affect fast MPL, but Im not so sure. Perhaps if I oversample ICM to the same degree it will fix itself magically.

changelog since 2/19/2026 
I2C communication now works just in time and everything is SD logged in time. Had to remove electromagnetic measurment, because it's easier to delete unnecessary measurment, than to fluff with adafruit libraries. Added LED test that is meant to simulate servomotor, code for which will be added in a final iteration soon(tm). Implemented a threshold metric that uses 2 values, to measure the top and bottom altitudes, after the top altitude is reached, we wait for the bottom one to trigger LED. This would imply descent. 

changelog since 2/24/2026 
https://github.com/oopCole added servo controls to the code instead of LED. Made OLED presence non-critical issue, so it can run without the display connected now. Also, changed code to be NASA 10 rules compliant and easier to read. Tested in the vacuum chamber and is proven to be working. 

changelog since 2/25/2026 
Changed the altimeter data gets from continuous to one-off with the manual DRDY reset. It is basically continouos, but Im doing it via manual register manipulation, instead of internal libraries and default settings. This actually yields unique altitude results every ~60ms (the 30ms datapoint doubles) except unique data every 1 second as it was before with continuous mode. Also, added a note to excel to highlight when exactly servomotor is activated, it now shows time and altitude.
