var distance = 0x7fffffff;
const spawn = require('child_process').spawn;
const car = spawn('/root/project/raspberry-pi-car/distance');
car.stdout.on("data", (data) => 
{
    distance = data.toString();
    // console.log(distance+"\n");
});
exports.distance = distance;
exports.doDistanceCtrl = function(app) {

    /**
     *  doDistance
     */
    app.get("/api/distance", function(req, res) 
    {
        var message = {
            'distance': distance
        }
        if (!req.session.user) {
            var message = {
                'stat': 'deny'
            }
            res.send(200, message);
            return false;
        }
        res.send(200,message);
    });
    app.get("/api/setDistance", function(req, res) 
    {
        distance = req.query.distance;
    });
};