This library for Arduino is intended to provide functions for connecting to and controlling ATEM video switchers from Blackmagic Design (https://www.blackmagicdesign.com/products/atem/). They make awesome HD video switchers, really cheap ones too and they are controlled over IP! So while you can get away with using their nice Win/Mac app and switch video professionally, you can also shell out a lot of money on a hardware interface which the broadcast pros will prefer any day. Hey, it's a win for everyone!

Please check out http://skaarhoj.com/ for our products based on this library.

ATEMbase: This is the super class that just does connection basics. See sub libraries such as ATEMstd, ATEMmax, ATEMmin etc.

Please see the API documentation at http://skaarhoj.com/fileadmin/BMDPROTOCOL.html

GPL licensed:
The library is licensed under GNU GPL v3. It allows you to use the library for any project, even commercial ones, as long as you keep the code using the library open - and deliver a copy to your client. In other words, even though you might deliver a black box hardware device, you still must give your client a copy of the Arduino sketch you have uploaded to the board. And how knows; either they will improve your product, maybe do nothing at all - or mess it up so you can sell some support hours. :-)

_kasper

# Modifications by Aron N. Het Lam
- Added support for the ESP32 WiFi module 
