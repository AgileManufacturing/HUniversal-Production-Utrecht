/*
 *  Copyright [2006-2007] [Stefan Kleine Stegemann]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package org.openbbs.blackboard;

import org.apache.commons.lang.Validate;

/**
 * Selects exactly one zone.
 *
 * @author stefan
 */
public class ExactZoneSelector implements ZoneSelector
{
   private Zone zone = null;

   public ExactZoneSelector(Zone zone) {
      Validate.notNull(zone);
      this.zone = zone;
   }

   public boolean selects(Zone zone) {
      return this.zone.equals(zone);
   }
}
