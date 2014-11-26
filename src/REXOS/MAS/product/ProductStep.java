package MAS.product;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class ProductStep {

	private int index;
	private String service;
	private Map<String, Object> criteria;

	public ProductStep(int index, String service, Map<String, Object> criteria) {
		this.index = index;
		this.service = service;
		this.criteria = criteria;
	}

	public int getIndex() {
		return index;
	}

	public String getService() {
		return service;
	}

	public Map<String, Object> getCriteria() {
		return criteria;
	}
	
	public JSONObject getCriteriaasJSON() {
		JSONObject returnvalue = new JSONObject(this.criteria);
		return returnvalue;
	}
	
	public void setCriteria(JSONObject newcriteria) {
		try {
			this.criteria = jsonToMap(newcriteria);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			System.out.println("Setting criteria failed");
		}
	}
	
	public static Map jsonToMap(JSONObject json) throws JSONException {
        Map<String, Object> retMap = new HashMap<String, Object>();

        if(json != JSONObject.NULL) {
            retMap = toMap(json);
        }
        return retMap;
    }

    public static Map toMap(JSONObject object) throws JSONException {
        Map<String, Object> map = new HashMap<String, Object>();

        Iterator<String> keysItr = object.keys();
        while(keysItr.hasNext()) {
            String key = keysItr.next();
            Object value = object.get(key);

            if(value instanceof JSONArray) {
                value = toList((JSONArray) value);
            }

            else if(value instanceof JSONObject) {
                value = toMap((JSONObject) value);
            }
            map.put(key, value);
        }
        return map;
    }

    public static List toList(JSONArray array) throws JSONException {
        List<Object> list = new ArrayList<Object>();
        for(int i = 0; i < array.length(); i++) {
            Object value = array.get(i);
            if(value instanceof JSONArray) {
                value = toList((JSONArray) value);
            }

            else if(value instanceof JSONObject) {
                value = toMap((JSONObject) value);
            }
            list.add(value);
        }
        return list;
    }

	@Override
	public String toString() {
		return String.format("(%d=%s,%s)", index, service, criteria);
	}
}