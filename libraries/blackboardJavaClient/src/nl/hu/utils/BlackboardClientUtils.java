package nl.hu.utils;

import java.util.Map;

public class BlackboardClientUtils {

	public static Object get(Map map, String pattern)  {
		String[] subpattern = pattern.split("\\.");
		Map<String, Object> recursiveMap = map;

		// check if a value can be returned directly
		if (subpattern.length == 1) {
			return recursiveMap.get(subpattern[0]);
		}

		for (int i = 0; i < subpattern.length; i++) {

			// if last subpatern is reached return the value
			if (subpattern.length - 1 == i) {
				return recursiveMap.get(subpattern[i]);
			}
			// if exception is thrown the patern did not exist
			try 
			{
				// create a new map for from the current subpatern
				recursiveMap = ((Map) recursiveMap.get(subpattern[i]));
				
				// if map is null the subpattern did not exist
				if (recursiveMap == null) 
				{
					return null;
				}
			} catch (Exception e) {
				return null;
			}
		}

		return map;
	}

}
