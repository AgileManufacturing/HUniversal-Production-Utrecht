function receivedMessage(received){
	if ( !received.hasOwnProperty('subject') ) {
		var notificator = document.getElementById('notificator');
		notificator.innerHTML += '<li class="list-group-item list-group-item-' + received.type + '">' + received.message + '</li>';
	}
}