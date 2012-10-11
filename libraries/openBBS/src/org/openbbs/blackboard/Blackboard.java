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

import java.util.Set;

/**
 * @author stefan
 */
public interface Blackboard
{
   /**
    * Open a new zone. Before a zone is not opened, it is not possible
    * to write entries to it.
    */
   public void openZone(Zone zone);

   /**
    * Close an open zone and remove all entries in this zone from the
    * receiver. Observers are not notified about the removal of entries.
    */
   public void closeZone(Zone zone);

   
   public boolean isZoneOpen(Zone zone);  
   
   /**
    * Write a non-null entry into a zone on the blackboard.
    */
   public void write(Zone zone, Object entry);

   /**
    * Determine the zone of a particular entry.
    */
   public Zone zoneOf(Object entry);

   /**
    * Return the first entry which lives in a selected zone and
    * is selected by the specified filter.
    */
   public Object read(ZoneSelector zoneSelector, EntryFilter filter);

   /**
    * Return all entries which are living in a selected zone and
    * which are selected by the specified filter.
    */
   public Set<Object> readAll(ZoneSelector zoneSelector, EntryFilter filter);

   /**
    * Check whether the receiver contains at least one entry which
    * lives in a selected zone and is selected by the specified filter.
    */
   public boolean exists(ZoneSelector zoneSelector, EntryFilter filter);

   /**
    * Remove and return the first object which lives in a selected
    * zone and is selected by the specified filter from the receiver.
    */
   public Object take(ZoneSelector zoneSelector, EntryFilter filter);

   /**
    * Register an observer which get's notified when the receiver's
    * state changes inside a selected zone.
    */
   public void registerInterest(ZoneSelector zoneSelector, BlackboardObserver observer);
}
