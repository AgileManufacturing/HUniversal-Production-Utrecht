<script src="websocket/dashboard_receiver.js"></script>
<script>
	function view_services(id){		
		$( "#services_"+id ).animate({
			marginTop: "-200px",
			height:"200px"
		}, 500 ,"linear", function(){
			console.log('finished');
			document.getElementById('button_'+id).onclick = function() { 
				view_image(id); 
			};
		});
	}
	function view_image(id){		
		console.log(id);
		$( "#services_"+id ).animate({
			marginTop: "0px",
			height: "0px"
		}, 500 ,"linear", function(){
			console.log('finished');
			document.getElementById('button_'+id).onclick = function() { 
				view_services(id); 
			};
		});
	}
</script>

<div id="equiplet_container" class="row">
</div>