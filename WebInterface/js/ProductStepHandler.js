function ProductStepHandler (service) {
    this.service = service;
    this.productSteps = new Array();
	this.properties = new Array();
	this.properties.target = new Array();
	this.properties.subjects = new Array();
	
	this.insertProductStep = function(index,serviceID){
		this.productSteps[index] = new ProductStep(this.service, index, serviceID);
		
		var properties = owl.copy(this.properties);
		this.productSteps[index].criteria = properties;
		
		var target = owl.copy(this.properties.target);
		this.productSteps[index].criteria.target = target;
		var subjects = owl.copy(this.properties.subjects);
		this.productSteps[index].criteria.subjects = subjects;
	}
	this.removeProductStep = function(index){
		this.productSteps.splice(index, 1);
	}
	
	this.setProperty = function(property, value){
		this.properties[property] = value;
	}
	this.getProperty = function(property){
		return this.properties[property];
	}
	this.removeProperty = function(property){
		delete this.properties[property];
	}
	
	this.toJSON = function(){
		var json = ' ';
		var currentProductStep = 0;
		for (var index in this.productSteps){
			if (currentProductStep != 0){ 
				json += ',';
			}
			json += this.productSteps[index].toJSON();
			currentProductStep++;
		}
		return json + '}';
	}
}
