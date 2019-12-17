const functions = require("firebase-functions");
const https = require("https");

// // Create and Deploy Your First Cloud Functions
// // https://firebase.google.com/docs/functions/write-firebase-functions
//
exports.helloWorld = functions.https.onRequest((request, response) => {
  response.send("Hello from Firebase!");
});

exports.loraReq = functions.https.onRequest((request, response) => {
  var data = {};
  data.team = 32;
  data.latitude = Number(request.body.DevEUI_uplink.LrrLAT);
  data.longitude = Number(request.body.DevEUI_uplink.LrrLON);
  data.timestamp = request.body.DevEUI_uplink.Time;

  data = JSON.stringify(data);

  const options = {
    hostname: "tgr2020-quiz.firebaseio.com",
    path: "/quiz/location/team32.json",
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "Content-Length": data.length
    }
  };

  console.log(`send: ${data}`);

  const req = https.request(options, res => {
    console.log(`statusCode: ${res.statusCode}`);

    res.on("data", d => {});
  });

  req.on("error", error => {
    console.error(error);
  });

  req.write(data);
  req.end();

  response.send(data);
});
