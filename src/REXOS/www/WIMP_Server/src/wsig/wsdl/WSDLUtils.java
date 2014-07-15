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

package wsig.wsdl;

import jade.content.onto.OntologyException;
import jade.content.schema.AgentActionSchema;
import jade.content.schema.Facet;
import jade.content.schema.ObjectSchema;
import jade.content.schema.facets.CardinalityFacet;
import jade.content.schema.facets.TypedAggregateFacet;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Map;

import javax.wsdl.Binding;
import javax.wsdl.BindingInput;
import javax.wsdl.BindingOperation;
import javax.wsdl.BindingOutput;
import javax.wsdl.Definition;
import javax.wsdl.Input;
import javax.wsdl.Message;
import javax.wsdl.Operation;
import javax.wsdl.Output;
import javax.wsdl.Part;
import javax.wsdl.Port;
import javax.wsdl.PortType;
import javax.wsdl.Service;
import javax.wsdl.Types;
import javax.wsdl.WSDLElement;
import javax.wsdl.WSDLException;
import javax.wsdl.extensions.ExtensionRegistry;
import javax.wsdl.extensions.schema.Schema;
import javax.wsdl.extensions.soap.SOAPAddress;
import javax.wsdl.extensions.soap.SOAPBinding;
import javax.wsdl.extensions.soap.SOAPBody;
import javax.wsdl.extensions.soap.SOAPOperation;
import javax.wsdl.factory.WSDLFactory;
import javax.wsdl.xml.WSDLWriter;
import javax.xml.namespace.QName;

import org.apache.axis.utils.bytecode.ParamReader;
import org.eclipse.xsd.XSDAnnotation;
import org.eclipse.xsd.XSDComplexTypeDefinition;
import org.eclipse.xsd.XSDCompositor;
import org.eclipse.xsd.XSDElementDeclaration;
import org.eclipse.xsd.XSDFactory;
import org.eclipse.xsd.XSDModelGroup;
import org.eclipse.xsd.XSDParticle;
import org.eclipse.xsd.XSDSchema;
import org.eclipse.xsd.XSDTypeDefinition;
import org.eclipse.xsd.impl.XSDComplexTypeDefinitionImpl;

import com.ibm.wsdl.BindingImpl;
import com.ibm.wsdl.BindingInputImpl;
import com.ibm.wsdl.BindingOperationImpl;
import com.ibm.wsdl.BindingOutputImpl;
import com.ibm.wsdl.InputImpl;
import com.ibm.wsdl.MessageImpl;
import com.ibm.wsdl.OperationImpl;
import com.ibm.wsdl.OutputImpl;
import com.ibm.wsdl.PartImpl;
import com.ibm.wsdl.PortImpl;
import com.ibm.wsdl.PortTypeImpl;
import com.ibm.wsdl.ServiceImpl;
import com.ibm.wsdl.TypesImpl;
import wsig.WSIGConfiguration;
import wsig.store.TypedAggregateSchema;

public class WSDLUtils {
	
	private static final String INPUT = "INPUT";
	private static final String OUTPUT = "OUTPUT";
	
	// -----------------------------------------------------------------------------
	// Package-scope methods

	static XSDSchema createSchema(String tns) {
		System.out.println("NAMESPACE: " +tns);
		XSDFactory xsdFactory = XSDFactory.eINSTANCE;
		XSDSchema xsd = xsdFactory.createXSDSchema();
		xsd.setSchemaForSchemaQNamePrefix(WSDLConstants.XSD);
		xsd.setTargetNamespace(tns);

		Map qNamePrefixToNamespaceMap = xsd.getQNamePrefixToNamespaceMap();
		qNamePrefixToNamespaceMap.put(WSDLConstants.XSD, xsd.getTargetNamespace());
		qNamePrefixToNamespaceMap.put(xsd.getSchemaForSchemaQNamePrefix(), WSDLConstants.XSD_URL);
		qNamePrefixToNamespaceMap.put(WSIGConfiguration.getInstance().getLocalNamespacePrefix(), tns);

		// Add annotation (without this xsd.getElement() is null)
		XSDAnnotation xsdAnnotation = xsdFactory.createXSDAnnotation();
		xsd.getContents().add(xsdAnnotation);
		xsdAnnotation.createUserInformation(null);
		
		return xsd;
	}

	static XSDTypeDefinition getComplexType(XSDSchema schema, String targetNameSpace, String typeName) {
		XSDTypeDefinition result = null;
		for (XSDTypeDefinition type : schema.getTypeDefinitions()) {
			if (type.hasNameAndTargetNamespace(typeName, targetNameSpace)) {
				result = type;
				break;
			}
		}
		return result;
	}

	static XSDComplexTypeDefinition createComplexType(String tns, String name) {
		XSDComplexTypeDefinition complexType = (XSDComplexTypeDefinitionImpl) XSDFactory.eINSTANCE.createXSDComplexTypeDefinition();
		if (name != null) {
			complexType.setName(name);
		}
		if (tns != null) {
			complexType.setTargetNamespace(tns);
		}

		return complexType;
	}

	static XSDElementDeclaration createElement(String tns, String name) {
		XSDElementDeclaration element = XSDFactory.eINSTANCE.createXSDElementDeclaration();
		if (name != null) {
			element.setName(name);
		}
		if (tns != null) {
			element.setTargetNamespace(tns);
		}
		
		return element;
	}
	
	static XSDComplexTypeDefinition addComplexTypeToSchema(String tns, XSDSchema schema, String complexTypeName) {
		XSDComplexTypeDefinition complexType = createComplexType(tns, complexTypeName);
		schema.getContents().add(complexType);

		return complexType;
	}

	static XSDComplexTypeDefinition addComplexTypeToElement(XSDElementDeclaration element) {
		XSDComplexTypeDefinition complexType = createComplexType(null, null);
		element.setAnonymousTypeDefinition(complexType);

		return complexType;
	}
	
	static XSDElementDeclaration addElementToSchema(String tns, XSDSchema schema, String elementName) {
		
		XSDElementDeclaration element = createElement(null, elementName);
		schema.getContents().add(element);
		
		return element;
	}
	
	static XSDModelGroup addSequenceToComplexType(XSDComplexTypeDefinition complexTypeDefinition) {

		XSDParticle particle = XSDFactory.eINSTANCE.createXSDParticle();
		XSDModelGroup contentSequence = XSDFactory.eINSTANCE.createXSDModelGroup();
		contentSequence.setCompositor(XSDCompositor.SEQUENCE_LITERAL);
		particle.setContent(contentSequence);
		complexTypeDefinition.setContent(particle);
		
		return contentSequence;
	}

	static XSDParticle addElementToSequence(String tns, XSDSchema schema, String elementName, String elementType, XSDModelGroup sequence) {
		XSDElementDeclaration element = createElement(null, elementName);
		
		XSDTypeDefinition complexType;
		if (WSDLConstants.jade2xsd.values().contains(elementType)) {
			complexType = schema.resolveSimpleTypeDefinition(WSDLConstants.XSD_URL, elementType);
		} else {
			complexType = getComplexType(schema, tns, elementType);
			
			if (complexType == null) {
				// Not already present in definition type
				// Create a temp definition in my tns
				complexType = createComplexType(tns, elementType);
			}
		}
		element.setTypeDefinition(complexType);
		
		XSDParticle particle = XSDFactory.eINSTANCE.createXSDParticle();
		particle.setContent(element);
		sequence.getContents().add(particle);
		
		return particle;
	}

	static XSDParticle addElementToSequence(String tns, XSDSchema schema, String elementName, String elementType, XSDModelGroup sequence, Integer minOcc, Integer maxOcc) {
		XSDParticle particle = addElementToSequence(tns, schema, elementName, elementType, sequence);
		if (minOcc != null) {
			particle.setMinOccurs(minOcc);
		}
		if (maxOcc != null) {
			particle.setMaxOccurs(maxOcc);
		}
		return particle;
	}
	
	static Types createTypes(ExtensionRegistry registry, XSDSchema wsdlTypeSchema) throws WSDLException {
		Types types = new TypesImpl();

		Schema schema = (Schema) registry.createExtension(
							Types.class, new QName(WSDLConstants.XSD_URL,
							WSDLConstants.SCHEMA));
		schema.setElement(wsdlTypeSchema.getElement());
		types.addExtensibilityElement(schema);
		
		return types;
	}
	
	static String getLocalPart(String tns) {
		return tns.substring(tns.indexOf(":")+1);
	}
	
	static Definition createWSDLDefinition(WSDLFactory factory, String tns) {
		Definition definition = factory.newDefinition();
		definition.setQName(new QName(tns, getLocalPart(tns)));
		definition.setTargetNamespace(tns);
		definition.addNamespace(WSIGConfiguration.getInstance().getLocalNamespacePrefix(), tns);
		definition.addNamespace(WSDLConstants.XSD, WSDLConstants.XSD_URL);
		definition.addNamespace(WSDLConstants.XSI, WSDLConstants.XSI_URL);
		definition.addNamespace(WSDLConstants.WSDL_SOAP, WSDLConstants.WSDL_SOAP_URL);
		
		return definition;
	}
	
	static PortType createPortType(String tns) {
		PortType portType = new PortTypeImpl();
		portType.setUndefined(false);
		portType.setQName(new QName(getPortName(tns)));
		return portType;
	}
	
	static Binding createBinding(String tns) {
		Binding binding = new BindingImpl();
		PortType portType = new PortTypeImpl();
		portType.setUndefined(false);
		portType.setQName(new QName(tns, getPortName(tns)));
		binding.setPortType(portType);
		binding.setUndefined(false);
		binding.setQName(new QName(getBindingName(tns)));
		return binding;
	}

	static SOAPBinding createSOAPBinding(ExtensionRegistry registry, String soapStyle) throws WSDLException {
		SOAPBinding soapBinding = (SOAPBinding) registry.createExtension(
				Binding.class, new QName(WSDLConstants.WSDL_SOAP_URL, WSDLConstants.BINDING));
		soapBinding.setStyle(soapStyle);
		soapBinding.setTransportURI(WSDLConstants.TRANSPORT_URL);
		return soapBinding;
	}
	
	static Port createPort(String tns) {
		Binding bindingP = new BindingImpl();
		bindingP.setQName(new QName(tns, tns.substring(4)+WSDLConstants.BINDING_SUFFIX));
		bindingP.setUndefined(false);
		Port port = new PortImpl();
		port.setName(getPortName(tns));
		port.setBinding(bindingP);
		return port;
	}
	
	static SOAPAddress createSOAPAddress(ExtensionRegistry registry, String serviceName) throws WSDLException {
		SOAPAddress soapAddress = null;
		soapAddress = (SOAPAddress)registry.createExtension(Port.class,new QName(WSDLConstants.WSDL_SOAP_URL, "address"));		
		soapAddress.setLocationURI(WSIGConfiguration.getInstance().getWsigUri()+"/"+serviceName);
		return soapAddress;
	}
	
	static Service createService(String tns) {
		Service service = new ServiceImpl();
		service.setQName(new QName(getServiceName(tns)));
		return service;
	}

	static Operation createOperation(String actionName) {
		Operation operation = new OperationImpl();
		operation.setName(actionName);
		operation.setUndefined(false);
		return operation;
	}

	static BindingOperation createBindingOperation(ExtensionRegistry registry, String tns, String actionName) throws WSDLException {

		BindingOperation operationB = new BindingOperationImpl();
		operationB.setName(actionName);
		SOAPOperation soapOperation = (SOAPOperation) registry
				.createExtension(BindingOperation.class,
						new QName(WSDLConstants.WSDL_SOAP_URL, WSDLConstants.OPERATION));
		soapOperation.setSoapActionURI(getActionName(tns));
		operationB.addExtensibilityElement(soapOperation);
		return operationB;
	}

	static BindingInput createBindingInput(ExtensionRegistry registry, String tns, String soapUse) throws Exception{
		
		return (BindingInput)createBinding(registry, tns, soapUse, INPUT);
	}
	
	static BindingOutput createBindingOutput(ExtensionRegistry registry, String tns, String soapUse) throws Exception  {

		return (BindingOutput)createBinding(registry, tns, soapUse, OUTPUT);
	}

	static Message createMessage(String tns, String name) {
		Message messageIn = new MessageImpl();
		messageIn.setQName(new QName(tns, name));
		messageIn.setUndefined(false);
		return messageIn;
	}

	static Input createInput(Message messageIn) {
		Input input = new InputImpl();
		input.setMessage(messageIn);
		return input;
	}

	static Output createOutput(Message messageOut) {
		Output output = new OutputImpl();
		output.setMessage(messageOut);
		return output;
	}
	
	static Part createTypePart(String name, String type, String tns) {
		Part part = new PartImpl();
		String namespaceURI;
		if (WSDLConstants.jade2xsd.values().contains(type)) {
			namespaceURI = WSDLConstants.XSD_URL;
		} else {
			namespaceURI = tns;
		}
		
		QName qNameType = new QName(namespaceURI, type);
		part.setTypeName(qNameType);
		part.setName(name);
		return part;
	}
	
	static Part createElementPart(String name, String elementName, String tns) {
		QName qNameElement = new QName(tns, elementName);
		Part part = new PartImpl();
		part.setElementName(qNameElement);
		part.setName(name);
		return part;
	}
	
	static String getResponseName(String operationName) {
		return operationName+WSDLConstants.RESPONSE_SUFFIX;
	}

	static String getRequestName(String operationName) {
		return operationName+WSDLConstants.REQUEST_SUFFIX;
	}
	
	static int getAggregateCardMin(ObjectSchema containerSchema, String slotName) {
		
		int cardMin = 0;
		Facet[] facets = getAggregateFacet(containerSchema, slotName);
		for (Facet facet : facets) {
			if (facet instanceof CardinalityFacet) {
				cardMin = ((CardinalityFacet) facet).getCardMin();
			} 
		}
		
		return cardMin;
	}

	static int getAggregateCardMax(ObjectSchema containerSchema, String slotName) {
		
		int cardMin = 0;
		Facet[] facets = getAggregateFacet(containerSchema, slotName);
		for (Facet facet : facets) {
			if (facet instanceof CardinalityFacet) {
				cardMin = ((CardinalityFacet) facet).getCardMax();
			} 
		}
		
		return cardMin;
	}

	static void writeWSDL(WSDLFactory factory, Definition definition, String serviceName) throws Exception {
		String fileName = WSIGConfiguration.getInstance().getWsdlDirectory() + File.separator + serviceName + ".wsdl";
		WSDLWriter writer = factory.newWSDLWriter();
		File file = new File(fileName);
		PrintWriter output = new PrintWriter(file);
		writer.writeWSDL(definition, output);
	}
	
	
	
	// -----------------------------------------------------------------------------
	// Public methods
	
    public static String[] getParameterNames(Method method) {
        // Don't worry about it if there are no params.
        int numParams = method.getParameterTypes().length;
        if (numParams == 0)
            return null;

        // Get declaring class
        Class c = method.getDeclaringClass();
        
        // Don't worry about it if the class is a Java dynamic proxy 
        if(Proxy.isProxyClass(c)) {
            return null;
        }
        
        try {
            // Get a parameter reader
            ParamReader pr = new ParamReader(c);

            // Get the paramter names
            return pr.getParameterNames(method);
            
        } catch (IOException e) {
        	return null;
        }
    }
	
	public static String getResultName(String operationName) { 
		return operationName+WSDLConstants.RETURN_SUFFIX;
	}

	public static TypedAggregateSchema getTypedAggregateSchema(ObjectSchema containerSchema, String slotName) throws OntologyException  {
		String typeName = containerSchema.getSchema(slotName).getTypeName();
		ObjectSchema elementType = getAggregateElementSchema(containerSchema, slotName);
		return new TypedAggregateSchema(typeName, elementType);
	}	

	public static ObjectSchema getAggregateElementSchema(ObjectSchema containerSchema, String slotName) {
		
		ObjectSchema elementSchema = null;
		Facet[] facets = getAggregateFacet(containerSchema, slotName);
		for (Facet facet : facets) {
			if (facet instanceof TypedAggregateFacet) {
				elementSchema = ((TypedAggregateFacet) facet).getType();
			}
		}
		
		return elementSchema;
	}

	public static String getAggregateType(String elementType, String aggregateType) {
		// aggregateType (SET, SEQUENCE,...) no more used
		
		char[] uppercaseElementType = elementType.toCharArray();
		uppercaseElementType[0] = Character.toUpperCase(uppercaseElementType[0]);

		return WSDLConstants.ARRAY_OF + new String(uppercaseElementType);
	}

	public static URL getWsdlUrl(String serviceName) {
		URL wsdlUrl = null;
		try {
			wsdlUrl = new URL(WSIGConfiguration.getInstance().getWsigUri()+"/"+serviceName+"?WSDL");
		} catch (MalformedURLException e) {
		}
		return wsdlUrl;
	}

	
	
	
	
	// -----------------------------------------------------------------------------
	// Private methods
	
	private static WSDLElement createBinding(ExtensionRegistry registry, String tns, String soapUse, String type) throws Exception{

		SOAPBody soapBody;
		WSDLElement binding;
		if (INPUT.equals(type)) {
			binding = new BindingInputImpl();
		} else {
			binding = new BindingOutputImpl();
		}
		
		try {
			soapBody = (SOAPBody) registry.createExtension(BindingInput.class,
							new QName(WSDLConstants.WSDL_SOAP_URL, WSDLConstants.BODY));
		} catch (WSDLException e) {
			throw new Exception("Error in SOAPBodyInput Handling", e);
		}
		soapBody.setUse(soapUse);
		if (WSDLConstants.USE_ENCODED.equals(soapUse)) {
			ArrayList encodingStylesInput = new ArrayList();
			encodingStylesInput.add(WSDLConstants.ENCODING_URL);
			soapBody.setEncodingStyles(encodingStylesInput);
			soapBody.setNamespaceURI(tns);
		}
		binding.addExtensibilityElement(soapBody);
		return binding;
	}
	
	private static String getActionName(String tns) {
		return tns+WSDLConstants.ACTION_SUFFIX;
	}
	
	private static String getBindingName(String tns) {
		return getLocalPart(tns)+WSDLConstants.BINDING_SUFFIX;
	}

	private static String getServiceName(String tns) {
		return getLocalPart(tns)+WSDLConstants.SERVICE_SUFFIX;
	}
	
	private static String getPortName(String tns) {
		return getLocalPart(tns)+WSDLConstants.PORT_TYPE_SUFFIX;
	}

	private static Facet[] getAggregateFacet(ObjectSchema containerSchema, String slotName) {
		
		Facet[] facets = containerSchema.getFacets(slotName);
		
		// If there are no facets get result facets
		if ((facets == null || facets.length == 0) && containerSchema instanceof AgentActionSchema) {
			facets = ((AgentActionSchema)containerSchema).getResultFacets();
		}
		
		return facets;
	}
}
