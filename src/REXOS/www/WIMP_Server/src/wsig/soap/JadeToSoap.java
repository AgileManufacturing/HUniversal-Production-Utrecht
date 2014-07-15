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

package wsig.soap;

import jade.content.abs.AbsAggregate;
import jade.content.abs.AbsPrimitive;
import jade.content.abs.AbsTerm;
import jade.content.onto.BasicOntology;
import jade.content.schema.AgentActionSchema;
import jade.content.schema.AggregateSchema;
import jade.content.schema.ConceptSchema;
import jade.content.schema.ObjectSchema;
import jade.content.schema.PrimitiveSchema;

import javax.xml.soap.MessageFactory;
import javax.xml.soap.Name;
import javax.xml.soap.SOAPBody;
import javax.xml.soap.SOAPElement;
import javax.xml.soap.SOAPEnvelope;
import javax.xml.soap.SOAPMessage;
import javax.xml.soap.SOAPPart;

import org.apache.log4j.Logger;

import wsig.WSIGConfiguration;
import wsig.WSIGConstants;
import wsig.store.ActionBuilder;
import wsig.store.WSIGService;
import wsig.wsdl.WSDLConstants;
import wsig.wsdl.WSDLUtils;

public class JadeToSoap {

	private static Logger log = Logger.getLogger(JadeToSoap.class.getName());
	
	private SOAPEnvelope envelope;
	private String tns;
	private String localNamespacePrefix;
	private String soapStyle;
	
	public JadeToSoap() {
		localNamespacePrefix = WSIGConfiguration.getInstance().getLocalNamespacePrefix(); 
		soapStyle = WSIGConfiguration.getInstance().getWsdlStyle();
	}

	public SOAPMessage convert(AbsTerm resultAbsObject, WSIGService wsigService, String operationName) throws Exception {
	
		// Get tns
		tns = WSDLConstants.URN + ":" + wsigService.getServicePrefix() + wsigService.getServiceName();
			
		// Create soap message
        MessageFactory messageFactory = MessageFactory.newInstance();
        SOAPMessage soapResponse = messageFactory.createMessage();
        
        // Create soap part and body            
        SOAPPart soapPart = soapResponse.getSOAPPart();
        envelope = soapPart.getEnvelope();
        envelope.setPrefix(WSDLConstants.SOAPENVELOP_PREFIX);
        envelope.addNamespaceDeclaration(WSDLConstants.XSD, WSDLConstants.XSD_URL);
        SOAPBody body = envelope.getBody();

        // Create soap element
        String responseElementName = operationName + WSDLConstants.RESPONSE_SUFFIX;
        SOAPElement responseElement = addSoapElement(body, responseElementName, localNamespacePrefix, null, tns);
        
        // Get action builder
        log.debug("Operation name: "+operationName);
        ActionBuilder actionBuilder = wsigService.getActionBuilder(operationName);
        if (actionBuilder == null) {
			throw new Exception("Action builder not found for operation "+operationName+" in WSIG");
        }
        
        // Get action schema
        AgentActionSchema actionSchema = actionBuilder.getOntologyActionSchema();;

		// Get result schema
        ObjectSchema resultSchema = actionSchema.getResultSchema();
        if (resultSchema != null) {
        	log.debug("Ontology result type: "+resultSchema.getTypeName());

        	// Create soap message
            convertObjectToSoapElement(actionSchema, resultSchema, resultAbsObject, WSDLUtils.getResultName(operationName), responseElement);
        } else {
        	log.debug("Ontology with no result type");
        }

        // Save all modifies of soap message
        soapResponse.saveChanges();
        
		return soapResponse;
	}

	private SOAPElement convertObjectToSoapElement(ObjectSchema containerSchema, ObjectSchema resultSchema, AbsTerm resultAbsObj, String elementName, SOAPElement rootSoapElement) throws Exception {
		
		SOAPElement soapElement = null;
		String soapType = null;
		ObjectSchema newContainerSchema = resultSchema;
		
		if (resultSchema instanceof PrimitiveSchema) {
			
			// PrimitiveSchema
			log.debug("Elaborate primitive schema: "+elementName+" of type: "+resultSchema.getTypeName());

			// Get type and create soap element
	        soapType = (String) WSDLConstants.jade2xsd.get(resultSchema.getTypeName());
			soapElement = addSoapElement(rootSoapElement, elementName, WSDLConstants.XSD, soapType, "");

			AbsPrimitive primitiveAbsObj = (AbsPrimitive)resultAbsObj;
			
	        // Create a text node which contains the value of the object.
	        // Format date objects in ISO8601 format;
	        // for every other kind of object, just call toString.
	        if (BasicOntology.DATE.equals(primitiveAbsObj.getTypeName())) {
	        	soapElement.addTextNode(WSIGConstants.ISO8601_DATE_FORMAT.format(primitiveAbsObj.getDate()));
	        } else {
	        	soapElement.addTextNode(primitiveAbsObj.toString());
	        }
		} else if (resultSchema instanceof ConceptSchema) {
			
			// ConceptSchema
			log.debug("Elaborate concept schema: "+elementName+" of type: "+resultSchema.getTypeName());

			// Get type and create soap element
	        soapType = resultSchema.getTypeName();
			soapElement = addSoapElement(rootSoapElement, elementName, localNamespacePrefix, soapType, "");
			
			// Elaborate all sub-schema of current complex schema 
			for (String conceptSlotName : resultSchema.getNames()) {
				ObjectSchema slotSchema = resultSchema.getSchema(conceptSlotName);
			
				// Get sub-object value 
				AbsTerm subAbsObject = (AbsTerm)resultAbsObj.getAbsObject(conceptSlotName);
				
				// Do recursive call
				convertObjectToSoapElement(newContainerSchema, slotSchema, subAbsObject, conceptSlotName, soapElement);
			}
		} else if (resultSchema instanceof AggregateSchema) {
			
			// AggregateSchema
			log.debug("Elaborate aggregate schema: "+elementName);

			// Get aggregate type
			ObjectSchema aggrSchema = WSDLUtils.getAggregateElementSchema(containerSchema, elementName);
			
			// Get slot type
			soapType = aggrSchema.getTypeName();
			if (aggrSchema instanceof PrimitiveSchema) {
				soapType = WSDLConstants.jade2xsd.get(soapType);
			}
			String itemName = soapType;
			String aggrType = resultSchema.getTypeName();
			soapType = WSDLUtils.getAggregateType(soapType, aggrType);
			
			// Create element
			soapElement = addSoapElement(rootSoapElement, elementName, localNamespacePrefix, soapType, "");
			
			// Elaborate all item of current aggregate schema 
			AbsAggregate aggregateAbsObj = (AbsAggregate)resultAbsObj;
			if (aggregateAbsObj != null) {
				for (int i=0; i<aggregateAbsObj.size(); i++) {
					
					//Get object value of index i
					AbsTerm itemObject = aggregateAbsObj.get(i);
	
					// Do recursive call
					convertObjectToSoapElement(newContainerSchema, aggrSchema, itemObject, itemName, soapElement);
				}
			}
		}
					
		return soapElement;
	}

	private SOAPElement addSoapElement(SOAPElement rootSoapElement, String elementName, String prefix, String soapType, String tns) throws Exception {

		// Create Name and Element
		String elementPrefix = "";
		if (WSDLConstants.STYLE_RPC.equals(soapStyle) && rootSoapElement instanceof SOAPBody) {
			elementPrefix = prefix;
		}
		Name soapName = envelope.createName(elementName, elementPrefix, tns);
	    SOAPElement soapElement = rootSoapElement.addChildElement(soapName);
	    
	    // Add encoding style only in result tag and for style rpc
        if (WSDLConstants.STYLE_RPC.equals(soapStyle) && rootSoapElement instanceof SOAPBody) {
        	soapElement.setEncodingStyle(WSDLConstants.ENCODING_URL);
        }

	    // Add type to element
	    if (WSDLConstants.STYLE_RPC.equals(soapStyle) && prefix != null && soapType != null) {
		    Name typeName = envelope.createName(WSDLConstants.TYPE, WSDLConstants.XSI, WSDLConstants.XSI_URL);
		    soapElement.addAttribute(typeName, prefix+":"+soapType);
	    }
	    
	    return soapElement;
	}
}
