var distance = 0x7fffffff;
exports.distance = distance;
exports.doDistanceCtrl = function(app) {

    /**
     *  doLogin
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
    app.get("/api/distance", function(req, res) 
    {
        distance = req.query.distance;
    });
};