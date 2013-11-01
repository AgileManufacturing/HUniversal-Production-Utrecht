/*
 * Globals
 */
var CURRENT_USER = "";
var currentApplication = 'undefined';

/*
 * Tab control
 */

function activateTab(pageID) {

	var home = document.getElementById('currentApplicationDiv');
	var settingsDiv = document.getElementById('settings');
	var progressDiv = document.getElementById('progressDiv');

	switch(pageID) {
		case 'home':

			$(home).css("display", "block");
			$(settingsDiv).css("display", "none");
			$(progressDiv).css("display", "none");
			hideCreateTab();
			setSelectedItem(pageID);

			fade('Carousel/index.html');
			break;

		case  'preferences':

			$(home).css("display", "none");
			$(progressDiv).css("display", "none");
			$(settingsDiv).css("display", "block");
			hideCreateTab();
			setSelectedItem(pageID);

			break;

		case 'progress':

			$(home).css("display", "none");
			$(settingsDiv).css("display", "none");
			$(progressDiv).css("display", "block");
			hideCreateTab();
			setSelectedItem(pageID);
			break;

		case 'create':
			createProductionCommand();
			break;

		default:
			alert("tab " + pageID + " not implemented yet");
			break;
	}
}

function fade(location) {
	var iframe = document.getElementById('currentApplication');

	$(iframe).css("display", "none");
	iframe.onload = function() {
		$(iframe).fadeIn(1500);
	}
	iframe.src = location;
}

function setSelectedItem(pageID) {
	var pageToSetSelected = document.getElementById(pageID);
	var tabs = document.getElementsByClassName("tab");

	for (var i = 0; i < tabs.length; i++) {
		if (tabs[i] == pageToSetSelected) {
			// tabs[i].style.backgroundColor = "#000000";
			tabs[i].style.backgroundColor = 'rgba(0, 0, 0, 0.3)';
		} else {
			tabs[i].style.backgroundColor = "#FFFFFF";
		}
	}
}

function hideCreateTab() {
	var create = document.getElementById('create');
	create.style.display = 'none';

	currentApplication = 'undefined';

	updateTabsLength();
}

function showCreateTab(showingApplication) {
	var create = document.getElementById('create');
	create.style.display = 'block';

	currentApplication = showingApplication;

	updateTabsLength();
}

function updateTabsLength() {
	var tabs = document.getElementsByClassName("tab");
	var length = tabs.length;

	for (var i = 0; i < tabs.length; i++) {
		if (tabs[i].style.display == 'none') {
			length--;
		}
	}

	for (var i = 0; i < tabs.length; i++) {
		var width = 100 / length;
		tabs[i].style.width = width + '%';
	}

	return length;
}

/*
 * Notification functions
 *
 * When using dismissMS = 0, the notification will not be removed automatically.
 * Types:
 * 'alert'
 * 'information'
 * 'error'
 * 'warning'
 * 'notification'
 * 'success'
 */

function showNotification(type, text, dismissMS) {
	var n = noty({
		text : text,
		type : type,
		dismissQueue : true,
		layout : 'bottomRight',
		theme : 'defaultTheme'
	});

	if (dismissMS != 0) {
		setTimeout(function() {
			$.noty.close(n.options.id);
		}, dismissMS);
	}
}

/*
 * Production functions
 */

function createProductionCommand() {

	switch(currentApplication) {
		case 'paint':
			createProductionCommandFromColorArray();
			window.parent.showNotification('success', 'Done', 3000);
			break;
		case 'pickAndPlace':
			createProductionCommandFromCrateObject();
			window.parent.showNotification('success', 'Done', 3000);
			break;
		case 'stacking':
			break;
		default:
			alert(currentApplication + " not yet implemented");
			break;
	}

}

function createProductionCommandFromCrateObject() {
	var frame = productionArray = document.getElementById("currentApplication").contentWindow;
	var cubeArray = frame.cubes;
	var crateRows = frame.CrateRows;
	var crateColumns = frame.CrateColumns;

	var cc = new pa_server.CommandContainer("CREATE_PA");

	cc.data = document.getElementsById("gwip").value + ":" + document.getElementById("gwport").value;

	for(var i = 0; i < cubes.length; i++) {
		if(cubes[i] === undefined) {
			continue;
		}

		var row = i / crateColumns;
		var color; // TODO-Duncan: Get color from ((Three.MeshLambertMaterial)cubes[i]).color ?
		var column = i % crateColumns;

		var step = new pa_server.ProductionStep({
			"id" : i,
			"type" : "Place",
			"row" : row,
			"column" : column,
			"color" : color
		});
		cc.payload.product.production.productionSteps.push(step);
	}
	cc.send();
	var xml = json2xml(cc.payload.product," ");
	cc.payload="";
	cc.command="SAVE_DATA";
	cc.data = xml;
	cc.send();
}

function createProductionCommandFromColorArray() {
	var frame = productionArray = document.getElementById("currentApplication").contentWindow;
	var pixels = frame.pixels;
	var columns = frame.columns;

	var cc = new pa_server.CommandContainer("CREATE_PA");

	cc.data = document.getElementById("gwip").value + ":" + document.getElementById("gwport").value;
	//cc.product = new pa_server.Product();

	//cc.product.productionSteps.push(pixels);

	var y = 0;
	var x = 0;
	for (var i = 0; i < pixels.length; i++) {
		if (pixels[i] === undefined)
			continue;

		y = arrayNumberToRow(i, columns);
		x = arrayNumberToColumn(i, columns);
		
		//make sure the PA receives doubles :| TODO: Remove. This should get fixed in the PA code
		y = y + 0.1;
		x = x + 0.1;
		
		var step = new pa_server.ProductionStep({
			"id" : i,
			"shapeName" : "dot",
			"color" : pixels[x],
			"x" : x,
			"y" : y
		});
		cc.payload.product.production.productionSteps.push(step);

		//myWindow = window.open("data:text/html," + encodeURIComponent(json2xml(cc, '')), "_blank", "width=200,height=100");
		//myWindow.focus();

	}
	cc.send();
	//console.log("\n " + JSON.stringify(cc.product));
	var xml = json2xml(cc.payload.product," ");
	cc.payload="";
	cc.command="SAVE_DATA";
	cc.data = xml;
	cc.send();

}

function startPADemo() {
	//initTabs();
	//var cc = new pa_server.CommandContainer("CREATE_PA");
	//cc.product = new pa_server.Product();
	var port = document.getElementById("gwport").value;
	var ip = document.getElementById("gwip").value;
	pa_server.sendString('{"id":0,"data":"' + ip + ':' + port + '","command":"CREATE_PA","product":{"_production":{"_productionSteps":[{"_requiredTimeSlots":0,"_id":1,"_capability":0,"_parameters":{"_parameterGroups":{"Shape":{"_name":"Shape","_parameters":{"Id":{"_key":"Id","_value":"8"}}},"loc":{"_name":"loc","_parameters":{"y":{"_key":"y","_value":"2"},"x":{"_key":"x","_value":"2"}}},"Color":{"_name":"Color","_parameters":{"Id":{"_key":"Id","_value":"7"}}}}}},{"_requiredTimeSlots":0,"_id":2,"_capability":1,"_parameters":{"_parameterGroups":{"Shape":{"_name":"Shape","_parameters":{"Id":{"_key":"Id","_value":"8"}}},"loc":{"_name":"loc","_parameters":{"y":{"_key":"y","_value":"2"},"x":{"_key":"x","_value":"2"}}},"Color":{"_name":"Color","_parameters":{"Id":{"_key":"Id","_value":"7"}}}}}},{"_requiredTimeSlots":0,"_id":3,"_capability":2,"_parameters":{"_parameterGroups":{"Shape":{"_name":"Shape","_parameters":{"Id":{"_key":"Id","_value":"8"}}},"loc":{"_name":"loc","_parameters":{"y":{"_key":"y","_value":"2"},"x":{"_key":"x","_value":"2"}}},"Color":{"_name":"Color","_parameters":{"Id":{"_key":"Id","_value":"7"}}}}}},{"_requiredTimeSlots":0,"_id":4,"_capability":3,"_parameters":{"_parameterGroups":{"Shape":{"_name":"Shape","_parameters":{"Id":{"_key":"Id","_value":"8"}}},"loc":{"_name":"loc","_parameters":{"y":{"_key":"y","_value":"2"},"x":{"_key":"x","_value":"2"}}},"Color":{"_name":"Color","_parameters":{"Id":{"_key":"Id","_value":"7"}}}}}}],"_prodletmap":{"_items":{"1":[],"2":[],"3":[],"4":[]}}}}}');
}

function startPAServer() {
	var cc = new pa_server.CommandContainer("START_PA_SERVER");
	cc.send();
}

/*
 * Other functions
 */

function arrayNumberToRow(number, columns) {
	return Math.floor(number / columns);
}

function arrayNumberToColumn(number, columns) {
	return number % columns;
}

