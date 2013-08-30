function login(username, password) {
	CURRENT_USER=username;
	var ret = pa_server
			.sendString('{"command":"LOGIN", "loginData":{"username":"'
					+ username + '","password":"' + password + '"}}');
	if (ret.error == true) {
		loginMessageAlert("No connection to server");
	}
}
function loginCallBack(msg) {
	console.log("loginCallBack: " + msg);
	if (msg == "LOGIN SUCCEEDED") {
		document.getElementById("loginMessageBox").style.display = 'none';
		document.getElementById("login").style.display = 'none';
		document.getElementById("mainUIDiv").style.display = 'block';
		showNotification('success', 'Welcome '+ CURRENT_USER, 3000);
	} else {
		CURRENT_USER="";
		loginMessageAlert("Please enter a correct username and password");
		document.getElementById("username").value = '';
		document.getElementById("password").value = '';
	}
}

function loginMessageAlert(msg) {
	document.getElementById("loginMessageBox").style.display = 'block';
	document.getElementById("loginMessageText").innerHTML = msg;
}