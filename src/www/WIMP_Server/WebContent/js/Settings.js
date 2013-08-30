var WS_LOCAL_URL = "ws://localhost:8080/WIMP_Server/ProductServlet";
var WS_IP = "145.89.191.131";
var WS_SERVER_URL = "ws://"+WS_IP+":8080/WIMP_Server/ProductServlet";
var WS_URL = "";//is set in init()

var DEFAULT_PAGW_SERVER = "localhost";
var DEFAULT_PAGW_PORT = 10081;

var PAINTAPP_ROWS = 30;
var PAINTAPP_COLUMNS = 30;



Settings = {};
		Settings.debug = false;
		Settings.localDebug =false;
		Settings.login = true;

Settings.setUp = function(){
	console.log("Settings.setUp");
	var params = location.search.substr(location.search.indexOf("?") + 1);
	var sval = "";
	params = params.split("&");
	for ( var i = 0; i < params.length; i++) {
		temp = params[i].split("=");
		console.log(temp[0] + " : " + temp[1]);
		if ([ temp[0] ] == "debug") {
			this.debug = (temp[1] === 'true');
		}
		else if ([ temp[0] ] == "localDebug"){
			this.localDebug = (temp[1] === 'true');
			if(this.localDebug)
				this.debug=true;
			
		}
		else if ([ temp[0] ] == "login"){
			this.login = (temp[1] === 'true');
		}
		
	}
	if (sval == "" )
		sval = false;

	return sval;

}