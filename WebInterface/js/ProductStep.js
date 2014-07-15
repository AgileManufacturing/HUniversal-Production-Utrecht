
Object.size = function(obj) {
    var size = 0, key;
    for (key in obj) {
        if (obj.hasOwnProperty(key)) size++;
    }
    return size;
};

function ProductStep (service, id,serviceID) {
	this.id = id;
    this.service = service;
	this.serviceID = serviceID;
    this.criteria = {};
	this.criteria.target = {};
	this.criteria.target.move = {};
	this.criteria.subjects = [];
	
	this.arrayToJSON = function(value,depth){
		var returnValue = "";
		var counter = 0;
		if( value instanceof Object ) {
			for (var prop in value) {
				if (value.hasOwnProperty(prop)) { 
					if( value[prop] instanceof Object ) {
						returnValue += '"' + prop + '"' + ':{';
						returnValue += this.arrayToJSON(value[prop],depth+1) + '';
						returnValue += '}';
					}
					else {
						returnValue += '"' + prop + '":"' + value[prop] + '"';
					}
					if (counter < Object.size(value)-1)
						returnValue += ',';
					returnValue += '';
					counter++;
			  	}
			}
		}
		else {
			return '"' + value + '"';
		}
		return returnValue;
	};
    this.toJSON = function() {
        return 	'{' + 
				'	"serviceID":' + this.serviceID + ',' +
				'	"id":' + this.id + ',' + 
				'	"service":"' + this.service + '",' + 
				'	"criteria": {' +
						this.arrayToJSON(this.criteria,2) + '' +
				'	}' +
				'}';
    };
}
