/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig.store;

import jade.core.AID;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.Map;

public class WSIGStore {

	private Map<String, WSIGService> services = new Hashtable<String, WSIGService>();

	
	public boolean isServicePresent(String serviceName) {
		return services.containsKey(serviceName);
	}
	
	public WSIGService getService(String serviceName) {
		return services.get(serviceName);
	}
	
	public void addService(String serviceName, WSIGService service) {
		services.put(serviceName, service);
	}
	
	public void removeService(String serviceName) {
		services.remove(serviceName);
	}
	
	public Collection<WSIGService> getServices(AID agentId) {
		
		Collection<WSIGService> aidServices = new ArrayList<WSIGService>();
		WSIGService service;
		Iterator<WSIGService> it = services.values().iterator();
		while(it.hasNext()) {
			service = it.next();
			if (service.getAid().equals(agentId)) {
				aidServices.add(service);
			}
		}
		return aidServices;
	}

	public Collection<WSIGService> getAllServices() {
		
		Collection<WSIGService> aidServices = new ArrayList<WSIGService>();
		aidServices.addAll(services.values());
		return aidServices;
	}
}
