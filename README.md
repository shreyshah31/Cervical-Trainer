# Cervical-Trainer (Mend)


Cervical Trainer

Physiotherapists find difficult to have an accurate assessment tool and databse system to track the recovery cycle of the
pateint, the physiotherapists also needs to motivate the patient everyday to execise. 
Hence we have made a light, portable, handy device which has long lasting battery, which is connected to an android app via bluetooth. The app has a Training mode which helps in training sessions of patient, for tracing the recovery cycle there is an inbuilt database system , and for motivating and engrosing them to exercise daily we have built gaming mode ,where the patients play a game and exercise on daily basis.


Device : 

Hardware Used:
MPU6050,HC-05(Bluetooth),Arduino Nano,Lipo Battery,
Battery charging Module.

Software Used:
Android Studio,Arduio IDE

Working:
Here initially MPU6050 and Bluetooth is interfaced with
Arduino Nano.The communication protocol used for reading 
data value from MPU6050 is I2C and for Bluetooth USART 
is used.

1. Here first of all the data from MPU6050 is taken through
I2C protocol. These data are know to be raw data values.
After the data received by the controller Using mathematical 
algorithm we convert this raw data into yaw , roll and pitch.

2. Charging module is used to charge the Lipo Battery.here when the
battery is charged for 1hr it would run continuously 24hrs.


3. After getting this processed data,then this data is then transmitted 
to available Mobile phone (Here the bluetooth must be enabled before
the transmission of data) via UsART protocol.

4. Then we have made an Android application where the received data
is used by the Software application where we have used the this data
for the excerise of patients via analaysis and game mode, also graph
mode is also available to track the patient exercise.

5. Inside Android Application we decrypt the data by using y,p,r as the breaking points,
the data is processed and then used for the movements in game and the training modes.

6. Inside training mode there are 3 buttons X Y And Z , this button will help the patient to concentrate the
movement in that specific direction.

7. The record button will help the doctor to record the specific data into the database,
and hence will reduce the pen and paper useage of the doctor.

8. There are three games, each game is ment for specific direction movement only,
For each game there is a setting page.

9. The setting page will have 3 attributes 1) Range 2)Time 3)Difficulty.
This will help the doctor and patient to understand all the three attribuites and its effect on daily basis.


