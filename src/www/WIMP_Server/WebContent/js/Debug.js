Debug = {};

Debug.log = function(msg, color) {

	if (DEBUG_ENABLED) {

		if (document.getElementById("debugDiv").style.display == 'none') {
			document.getElementById("debugDiv").style.display = 'block';

			
			this.log("debug div enabled");
		}

		console.log(msg);

		var dd = document.getElementById("debugDiv");
		dd.innerHTML = "&gt;&gt; " + msg + "<br />" + dd.innerHTML;

	}
};