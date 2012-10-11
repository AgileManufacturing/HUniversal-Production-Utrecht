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
 * Provides access to a blackboard for KnowledgeSources. Depending
 * on the implementation, access may be restricted to one or more
 * particular zones.
 * 
 * @author stefan
 */
public interface BlackboardAccess
{
   /**
    * Returns the first entry that is selected by the filter. Access
    * restrictions imposed by the BlackboardAccess apply.
    */
   public Object read(EntryFilter filter);

   /**
    * Returns all entries that are selected by the filter. Access
    * restrictions imposed by the BlackboardAccess apply.
    */
   public Set<Object> readAll(EntryFilter filter);

   /**
    * Returns true if the specified filter selects at least one
    * entry. Access restrictions imposed by the BlackboardAccess
    * apply.
    */
   public boolean exists(EntryFilter filter);

   /**
    * Remove and return the first entry that is selected by the filter.
    * Access restrictions imposed by the BlackboardAccess apply.
    */
   public Object take(EntryFilter filter);

   /**
    * Write a non-null entry to the blackboard. The zone to which the object
    * is written is determined by the BlackboardAccess.
    */
   public void write(Object entry);
}
