function receivedMessage(received){
	var equiplet_container = document.getElementById('equiplet_container');
	
	if(received.subject == 'update_equiplet'){
		if (document.getElementById('img_' + received.id) == null &&
			document.getElementById('equiplet_' + received.id) == null &&
			document.getElementById('services_' + received.id) == null &&
			document.getElementById('equiplet_status_' + received.id) == null &&
			document.getElementById('equiplet_mode_' + received.id) == null &&
			document.getElementById('button_' + received.id) == null){
			
			equiplet_container.innerHTML += 
				'<div class="col-xs-6 col-md-3" >' +
					'<a href="#" class="thumbnail">' +
						'<span id="equiplet_' + received.id + '" class="equiplet_id"><h4>Equiplet:</h4>' + received.id + '</span>' +
						'<img id="img_' + received.id + '" src="images/equiplet.png" alt="" style="height:200px;">' +
						'<div id="services_' + received.id + '" style="height:0px; background-color:white; position:relative; z-index:99; overflow:hidden;">' +
							'<b>Services:</b>' +
							'<ul id="service_list_' + received.id + '">' +
							'</ul><br />' +							
           					'<b>Product steps:</b><br />' +
            				'Currently planned: <span id="equiplet_planned_' + received.id + '"></span><br />' +
            				'Completed: <span id="equiplet_completed_' + received.id + '"></span><br />' +
            				'Failed: <span id="equiplet_failed_' + received.id + '"></span><br />' +
						'</div>' +
						'<span id="equiplet_status_' + received.id + '" class="equiplet_details left ' + received.status.type + '"></span>' +
						'<span id="equiplet_mode_' + received.id + '" class="equiplet_details right ' + received.mode.type + '"></span>' +
						'<span id="equiplet_state_' + received.id + '" class="equiplet_id" style="color:#AAA"></span>' +
						'<button id="button_' + received.id + '" type="button" class="equiplet_details_button btn btn-info" onclick="view_services(\'' + received.id + '\');">View details</button>' +
					'</a>' +
				'</div>';
		}
		document.getElementById('equiplet_status_' + received.id).innerHTML = 'Status: ' + received.status.content;
		document.getElementById('equiplet_mode_' + received.id).innerHTML = 'Mode: ' + received.mode.content;
		document.getElementById('equiplet_state_' + received.id).innerHTML = 'Running: ' + received.details.status;
		document.getElementById('equiplet_planned_' + received.id).innerHTML = received.details.plannedSteps;
		document.getElementById('equiplet_completed_' + received.id).innerHTML = received.details.successfulSteps;
		document.getElementById('equiplet_failed_' + received.id).innerHTML = received.details.failedSteps;
		
		var service_list = document.getElementById('service_list_' + received.id);
		var services = received.services.split(',');
		service_list.innerHTML = '';
		for (var i=0;i<services.length;i++){
			if (services[i] != ''){
				service_list.innerHTML += '<li>' + services[i] + '</li>';
			}
		}
		
			
		var notificator = document.getElementById('notificator');
		notificator.innerHTML += '<li class="list-group-item list-group-item-info">Equiplet "' + received.id + '" updated</li>';
	}
}