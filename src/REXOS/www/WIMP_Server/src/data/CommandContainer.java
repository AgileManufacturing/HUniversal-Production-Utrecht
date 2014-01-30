package data;

import newDataClasses.Product;

public class CommandContainer {

	/*
	 * private Long id; private String command; private Production production;
	 * 
	 * public Long getId() { return id; }
	 * 
	 * public String getCommand() { return command; }
	 * 
	 * public Production getProduction() { return production; }
	 * 
	 * public void setId(Long id) { this.id = id; }
	 * 
	 * public void setCommand(String command) { this.command = command; }
	 * 
	 * public void setProduction(Production production) { this.production =
	 * production; }
	 * 
	 * public String toString() { return ""; }
	 */

	private Product product;
	private String command;
	private String data;
	private LoginData loginData;
	
	public void setLoginData(LoginData ld){
		this.loginData = ld;
	}
	
	public void setData(String data){
		this.data = data;
	}

	public void setCommand(String command) {
		this.command = command;
	}

	public String getCommand() {
		return command;
	}

	public void setProduct(Product p) {
		this.product = p;
	}

	public Product getProduct() {
		return product;
	}
	public LoginData getLoginData(){
		return this.loginData;
	}
	
	public String getData(){
		if(this.data.equals(""))
			return "0";
		return this.data;
	}

}
