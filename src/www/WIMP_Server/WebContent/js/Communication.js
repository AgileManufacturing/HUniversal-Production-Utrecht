function Communication() {

	var self = this;
	var ws = null;

	self.sendString = function(msg) {
		var ret = {};
		ret.error = false;
		ret.message = "";
		
		if (ws != null) {
			ws.send(msg);
			return ret;
		} else{
			Debug.log("ERR: no connection");
			
			ret.error=true;
			ret.message = "NO CONNECTION";
			return ret;
		}
	};
	
	
	self.ProductionStep = function(obj) {
		//this.step = null;
		//this.colorCode = null;
		//this.shapeCode = null;
		//this.location = {};
		this.id = obj.id;
		
		this.parameters = {
            "parameterGroups": {
                "Shape": {
                    "name": obj.shapeName,
                    "parameters": {
                        "Id": {
                            "key": "Id",
                            "value": 0
                        }
                    }
                },
                "loc": {
                    "name": "loc",
                    "parameters": {
                        "y": {
                            "key": "y",
                            "value": obj.y
                        },
                        "x": {
                            "key": "x",
                            "value": obj.x
                        }
                    }
                },
                "Color": {
                    "name": obj.color,
                    "parameters": {
                        "Id": {
                            "key": "Id",
                            "value":0
                        }
                    }
                }
            }};
		
	};
    
    /*
     //demo product step
    self.ProductionStep = function(obj) {//TODO: revert back to original design for production steps (see above)
        //this.step = null;
        //this.colorCode = null;
        //this.shapeCode = null;
        //this.location = {};
        this.id = obj.id;
        this.capability = 3;
        
        this.parameters = {
            "startPosition" : {
                "x" : obj.x,
                "y" : obj.y
            },
            "endPosition" : {
                "x": obj.x,
                "y": obj.y
            }
        };
        /* "parameterGroups": {
         "Shape": {
         "name": obj.shapeName,
         "parameters": {
         "Id": {
         "key": "Id",
         "value": 0
         }
         }
         },
         "loc": {
         "name": "loc",
         "parameters": {
         "y": {
         "key": "y",
         "value": obj.y
         },
         "x": {
         "key": "x",
         "value": obj.x
         }
         }
         },
         "Color": {
         "name": obj.color,
         "parameters": {
         "Id": {
         "key": "Id",
         "value":0
         }
         }
         }
         }}; 
        
    };*/

	self.Production = function(){
		this.ID = "";
		this._productionSteps = [];
	}
	
	self.Product = function() {
		this.productionSteps = [];
		
		this._production = {};
	};

	self.CommandContainer = function(cmd) {

		this.id = 0;
		this.command = cmd;
		//this.product = {};
		
		this.payload = {
				//todo move to server
				"callback": {
		            "host": WS_IP,
		            "port": "8888",
		            "reportLevel": 0,
		            "protocol": "JSON"
		        },
		        
				"product": {
					"production":{"productionSteps":[]}
				}
		};

		this.send = function() {
			if (ws != null) {
				ws.send(JSON.stringify(this));
				Debug.log("Send: " + JSON.stringify(this));
			} else {
				Debug.log("No connection");
			}
		};

	};

	self.start = function() {
		Debug.log("Connecting to: " + WS_URL);
		ws = new WebSocket(WS_URL);

		onopen = function(evt) {

			Debug.log("Opened connection");
			/*
			 * document.getElementById("connect").disabled = "disabled";
			 * 
			 * var cc = new self.CommandContainer("START_AGENT"); var Product =
			 * new self.Product(); var pstep1 = new self.ProductionStep();
			 * 
			 * production.productionSteps.push(pstep1);
			 * 
			 * cc.production = production;
			 * 
			 * cc.id = 0; Debug.log(JSON.stringify(cc));
			 * ws.send("{'id':0,'command':'CREATE_PA','production':{'id':0,'productionSteps':[{'colorCode':'ff0ff','column':0,'row':0,'shapeCode':1},{'colorCode':'ff0ff','column':3,'row':1,'shapeCode':1},{'colorCode':'ff0ff','column':0,'row':1,'shapeCode':1},{'colorCode':'ff0ff','column':3,'row':2,'shapeCode':1},{'colorCode':'ff0ff','column':0,'row':3,'shapeCode':1},{'colorCode':'ff0ff','column':1,'row':3,'shapeCode':1},{'colorCode':'ff0ff','column':3,'row':4,'shapeCode':1}]}}");
			 */
		};

		ws.onmessage = function(evt) {

			var data = evt.data.replace(/\'/g, "\"");
			console.log("REC DATA: " + data);
			data = JSON.parse(data);

			if (data.error == true) {
				Debug.log("<span style='color:red'>onmessage error: "
						+ data.message + "</span>");
						showNotification("error", "Error: " + data.message, 3000) 
			} else if (data.error == false) {
				Debug.log("<span style='color:green'>onmessage: "
						+ data.message + "</span>");
			} else
				Debug.log("<span style='color:green'>onmessage: " + evt.data
						+ "</span>");
			
			if(data.module == "login" ){
				loginCallBack(data.message);
			}
			
			if(data.message == "CONNECTION ACCEPTED"){
				//init()
				//alert("connection accepted");
				//startPAServer();
                document.getElementById("connectDiv").style.display = 'none';
				showNotification("success", "Connected to server", 3000);
			}
			else if(data.message == "LOGIN SUCCEEDED"){
				document.getElementById('login').enabled = false;
			}
			else if (data.message.indexOf("Wrote XML to")!=-1){
				showNotification("success", data.message, 3000);
			}
		};

		ws.onclose = function(evt) {
			Debug.log("Connection closed");
			//showNotification("success", "Connected to server", 3000) ;
			ws = null;
		};

		ws.onerror = function(evt) {
			Debug.log("<span style='color:red'>onerror: " + JSON.stringify(evt) + "</span>");

			ws = null;
		};
	};

	self.stop = function() {
		if (ws) {
			Debug.log("Stopping pa_server...");

			data.command = "CLOSE";
			ws.send(JSON.stringify(data));

			document.getElementById("connect").onclick = "Communication.start()";
		}
	};
}

var pa_server;
