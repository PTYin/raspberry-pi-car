var config = require(__dirname + '/config.js');
var express = require('express');
var app = express();

// app.use(express.logger());
app.use(express.bodyParser());
app.use(express.cookieParser());
app.use(express.session({secret: config.cookieSecret}));
app.use(express.static(__dirname + '/webapp'));
app.use(app.router);

var start = require(__dirname + '/start.js');
start.createRoutes(app);

var webapp = require(__dirname + '/webappServing.js');
webapp.createRoutes(app);

app.listen(config.port);
console.log("Server started and listening on port " + config.port);