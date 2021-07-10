var mqtt = require('mqtt');
var mongodb = require('mongodb');
var mongodbClient = mongodb.MongoClient;
const fs = require('fs');

//MQTT settings
mqtt_options={
    clientId:"server",
    clean:true
};

mqtt_topics_list = [
    "/esp32cam/up",
];

//mongodb settings
const db_name = "images_db";
const collection_name = "esp32cam";
const uri = "mongodb://localhost:27017";
var mydb;
var collection;

mqttclient = mqtt.connect('mqtt://192.168.1.210:1883', mqtt_options);
mqttclient.subscribe(mqtt_topics_list);

mqttclient.on('connect', mqtt_connect_handler);
mqttclient.on('message', mqtt_message_handler);
mqttclient.on('error', mqtt_error_handler);

mongodbClient.connect(uri, { useNewUrlParser: true, useUnifiedTopology: true }, db_handler);

//handle incoming message
function mqtt_message_handler(topic, message, packet)
{ 
    var img_data;
    try
    {
        img_data = JSON.parse(message);
        collection.insertOne(img_data, function(err, res) {
            if (err) throw err;
            console.log("1 document inserted");
        });
        //var buf = Buffer.from(img_data["encoded_frame"], 'base64');
        //fs.writeFileSync('./img.jpg', buf);
    }
    catch (err)
    {
        console.log("\n\nERR:");
        console.log(err);
    }
}

//handle incoming connect
function mqtt_connect_handler()
{
    console.log("connected  " + mqttclient.connected)
}

// handle error
function mqtt_error_handler(error)
{
    console.log("Can't connect" + error);
    process.exit(1);
}

function db_handler(err, db)
{
    mydb = db.db(db_name);
    collection = mydb.collection(collection_name)
}
