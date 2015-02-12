//WebSocket variable
var WEBSOCKET_URL = "ws://127.0.0.1:8887";
var WEBSOCKET_RECEIVER = 'interface';
var WEBSOCKET_SUBJECT_CONNECTION = 'connection';
var ws;


//Start connection
window.onload = function(){
	if ("WebSocket" in window) {
		console.log("Websocket created ");
		ws = new WebSocket(WEBSOCKET_URL);
		ws.onopen = function(evt) { onOpen(evt) };
		ws.onclose = function(evt) { onClose(evt) };
		ws.onmessage = function(evt) { 
			onMessage(evt); 
		};
		ws.onerror = function(evt) { onError(evt) };
	}
	else{
		alert("WebSocket NOT supported by your Browser!");
	}
};

//Send message to WebSocket
function sendWebsocketMessage(text) {
	console.log("sending: "+text);
	try{
		ws.send(text);
	} catch(exception){
		console.log("Exception while sending message to server: " + exception);
	}
}


function onClose (evt){
 	console.log("websocket closed");	
}


function onError (evt){
 	console.log("websocket error: " + evt.data);
	var notificator = document.getElementById('notificator');
	notificator.innerHTML += '<li class="list-group-item list-group-item-danger">WebSocket Server connection error: ' + evt.data + '</li>'
	$( ".list-group-item" ).fadeOut( 10000, "linear", null );
}

//When connection is established
function onOpen (evt) {
	console.log("Websocket opened");
	sendWebsocketMessage('{"receiver":"' + WEBSOCKET_RECEIVER + '", "subject":"' + WEBSOCKET_SUBJECT_CONNECTION + '"}');
}

//Incomming messages
function onMessage (evt){
 	console.log(evt.data);	
	var received = JSON.parse(evt.data);
	if (received.receiver == WEBSOCKET_RECEIVER){
		console.log(received);
		if (received.subject == WEBSOCKET_SUBJECT_CONNECTION){
			var notificator = document.getElementById('notificator');
			notificator.innerHTML += '<li class="list-group-item list-group-item-success">Connection established</li>';
		}
		receivedMessage(received);
		
		$( ".list-group-item" ).fadeOut( 10000, "linear", null );
	}
}
