using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

using System.Net.Http;
using System.Threading.Tasks;
using Windows.Storage.Streams;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace App1
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        const byte UPDATEME = 0x42;
        Microsoft.Maker.RemoteWiring.RemoteDevice arduino;
        Microsoft.Maker.Serial.IStream connection;
        Microsoft.Maker.Firmata.UwpFirmata firmata;
        String macAddress = "";
        String arduinoString = "";
        String updateAllURL = "";
        const String serverUrl = "http://192.168.0.5:5000"; //useful for testing "http://httpbin.org/get?";

        public MainPage()
        {
            this.InitializeComponent();
        }

        private void sendSetupToArduino(String setupMsg) {

            DataWriter writer = new DataWriter();

             writer.WriteString(setupMsg);

             firmata.sendSysex(UPDATEME, writer.DetachBuffer());
            
            //byte b = 1;

            //invoke the sendSysex command with ALL_PINS_COMMAND and our data payload as an IBuffer
            //firmata.sendSysex(Microsoft.Maker.Firmata.SysexCommand.CAPABILITY_QUERY, new byte[] { b }.AsBuffer());
            firmata.flush();
            //arduino.digitalWrite(13, Microsoft.Maker.RemoteWiring.PinState.HIGH);
            System.Diagnostics.Debug.WriteLine("[" + setupMsg + "] SENT");
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {

        }

        private void TextBlock_SelectionChanged(object sender, RoutedEventArgs e)
        {

        }

        private void ConfirmButton_Click(object sender, RoutedEventArgs e)
        {
            //TODO missing textfield control.. if some data is missing..
            this.confirmButton.IsEnabled = false;
            this.progressBar.Value = 50;
 
            arduinoString = "" + this.wifiSSIDText.Text + ";";
            arduinoString += "" + this.wifiPwdText.Password + ";";
            arduinoString += "" + this.labelText.Text + ";";
            arduinoString = arduinoString.PadRight(50);
            SetupRemoteArduino();

            updateAllURL = serverUrl + "/updateAll?";
            updateAllURL += "label=" + labelText.Text + "&";
            updateAllURL += "threshold=" + thresholdText.Text + "&";
            updateAllURL += "position=" + positionText.Text;
            //multiSetupRequests();
        }

        private async Task multiSetupRequests() {
            bool failReq = false;
            System.Diagnostics.Debug.WriteLine("reqRegister");
            //register the device with the macaddress. Server create a new device if it doesn't exist
            try
            {
                await reqRegister();
                //TODO Server notify if the device exist: we can ask to the user, if he wants to recover the previous information or use new one.
                System.Diagnostics.Debug.WriteLine("reqUpdateAll");
                //than... send the update
                try {
                    await reqUpdateAll();
                } catch (Exception exc) {
                    System.Diagnostics.Debug.WriteLine("reqUpdateAll: failed");
                    failReq = true;
                }
            } catch (Exception exc) {
                System.Diagnostics.Debug.WriteLine("reqRegister: failed");
                failReq = true;
            }
            if(failReq)
                this.progressBar.Value = 0;
            else
                this.progressBar.Value = 100;
            this.confirmButton.IsEnabled = true;
            
        }

        private async Task reqUpdateAll() {
           // System.Diagnostics.Debug.WriteLine("Request: [" + updateAllURL + "]");
            await sendRequestAsync(updateAllURL);            
        }

        private async Task reqRegister() {
            String requestUrl = serverUrl + "/register?";
            requestUrl += "macAddr=" + macAddress;
            await sendRequestAsync(requestUrl);
        }

        private async Task sendRequestAsync(String requestUrl) {
            using (HttpClient client = new HttpClient())
            using (HttpResponseMessage response = await client.GetAsync(requestUrl))
            using (HttpContent content = response.Content)
            {
                // ... Read the string.
                string result = await content.ReadAsStringAsync();
                System.Diagnostics.Debug.WriteLine("Received ["+result+"]");
            }
        }

        public void SetupRemoteArduino()
        {
            //configuring Arduino usb/serial connection
            connection = new Microsoft.Maker.Serial.UsbSerial("VID_2341");// "PID_0804" for MKR1000.. not recognized.. //PID_0043 for arduino UNO: it works!
            firmata = new Microsoft.Maker.Firmata.UwpFirmata();
            arduino = new Microsoft.Maker.RemoteWiring.RemoteDevice(firmata);

            //Need to manage the connection due to sysexcallbackmanagement
            firmata.begin(connection);
            //registering callback
            arduino.DeviceReady += ArduinoReady;
            arduino.StringMessageReceived += Arduino_StringMessageReceived;
            arduino.DeviceConnectionFailed += Arduino_DeviceConnectionFailed;
            arduino.SysexMessageReceived += Arduino_SysexMessageReceived;
            connection.begin(57600, Microsoft.Maker.Serial.SerialConfig.SERIAL_8N1);                       
        }

        private void Arduino_SysexMessageReceived(byte command, DataReader message)
        {
            System.Diagnostics.Debug.WriteLine("SysexM Message " + command);
        }

        private void Arduino_DeviceConnectionFailed(string message)
        {
            System.Diagnostics.Debug.WriteLine("Device Connection Failed. Reason: " + message);
        }

        private void Arduino_StringMessageReceived(string message)
        {
            System.Diagnostics.Debug.WriteLine("Arduino Message String [" + message + "]");
            if (message.StartsWith("MAC;")) { //parsing the mac address 
                Boolean macAddrField = false;
                for (int i = 0; i < message.Length; i++) {
                    if (message[i] == ';') {
                        macAddrField = !macAddrField; //enough for a single field...
                    } else if (macAddrField) {
                        this.macAddress = this.macAddress + message[i];
                    }
                }
                updateAllURL += "&macAddr=" + macAddress;
                multiSetupRequests();                
            }
        }

        public void ArduinoReady()
        {
            System.Diagnostics.Debug.WriteLine("Arduino Connection Ready");

            sendSetupToArduino(arduinoString);
        }

        private void TextBlock_SelectionChanged_1(object sender, RoutedEventArgs e)
        {

        }
    }
}
