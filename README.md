# midterm2

將mbed板與uLCD接上電腦

1. 開 new terminal

2. mkdir -p ~/ee2405new

3. cp -r ~/ee2405/mbed-os ~/ee2405new

4. cd ~/ee2405new

5. mbed compile --library --no-archive -t GCC_ARM -m B_L4S5I_IOT01A --build ~/ee2405new/mbed-os-build2

6. 開 new terminal (此terminal以terminal1代稱)

7. cd ~/HW3/src/model_depoly

8. sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -ff

9. teriminal1 會顯示wifi連線，與ip位址。

10. 開 new terminal (此terminal以terminal2代稱)

11. cd ~/HW3/src/model_depoly

12. sudo python3 wifi_mqtt/mqtt_client.py

13. terminal2 會顯示會顯示wifi連線，與ip位址，再來傳五次訊息至terminal1，terminal1會顯示接收到

14. 在terminal1 輸入 /1/run，按下enter，螢幕上會有 "Set up successful...\n"，這時候揮動mbed板，就會將判斷到的手勢在terminal1上顯示

15. 累積判斷10個手勢之後會結束程式，然後將model偵測的手勢傳到terminal2

 因為我打MQTT時，想要將我所分析的資料，傳遞到terminal2時會有BUG，但礙於時間我已經上傳了，只要將419行與524行註解，程式就可以正常運作到，將10筆model偵測的手勢傳到terminal2。
 10筆model偵測的手勢和與分析的結果，都有利用全域變數儲存，我剩下尚未實做的部分應該只剩下6-2、7-1和7-2。
