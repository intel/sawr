// Simple robot control web server based on express and roslib

var express = require('express');
var app = express();
var roslib = require('roslib');
var ros = new roslib.Ros();

// use port given on command line, otherwise use default
var port = process.argv[2] || 8080; 

// Serve static resources
app.use('/',express.static('../assets/static'));
app.use('/scripts',express.static('../assets/scripts'));

// Test basic web interfaces on "test" resource
app.all('/test', function(req,res,next) {
  // console.log('TEST parameters: ',req);
  next();  // go to next handler
})
app.get('/test', function(req,res) {
  res.send('TEST: GET')
})
app.post('/test', function(req,res) {
  res.send('TEST: POST')
})
app.put('/test/:name', function(req,res) {
  res.send('TEST: PUT')
  console.log('should create ',req.param["name"])
})
app.delete('/test/:name', function(req,res) {
  res.send('TEST: DELETE')
  console.log('should delete ',req.param["name"])
})

app.get('/params', function(req,res) {
  (function (local_res) {
    ros.getParams(function(params) {
      local_res.send(params)
    });
  }(res));
});

app.get('/longpoll', function(req,res) {
  (function (local_res) {
    setTimeout(function() {
      local_res.send("DONE!")
    },5000);
  }(res));
});

// Start server 
app.listen(port, function() {
  ros.getParams(function(params) {
    console.log('Params:',params)
  });
  console.log('Server listening on port ',port)
})
