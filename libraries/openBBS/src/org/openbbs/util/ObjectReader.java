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
package org.openbbs.util;

import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;

import org.apache.commons.lang.Validate;

/**
 * Helps reading objects from an ObjectInputStream.
 */
public class ObjectReader
{
   private ObjectInputStream inputStream = null;
   private boolean streamIsEmpty = false;

   public ObjectReader(File file) throws IOException {
      Validate.notNull(file);
      try {
         this.inputStream = new ObjectInputStream(new FileInputStream(file));
      }
      catch (EOFException _) {
         this.streamIsEmpty = true;
      }
      catch (FileNotFoundException _) {
         this.streamIsEmpty = true;
      }
   }

   public ObjectReader(ObjectInputStream inputStream) {
      Validate.notNull(inputStream);
      this.inputStream = inputStream;
   }

   public void readObjects(Delegate delegate) throws Exception {
      Validate.notNull(delegate);

      if (this.streamIsEmpty) {
         return;
      }

      boolean eof = false;
      try {
         while (!eof) {
            try {
               Object object = this.inputStream.readObject();
               delegate.didReadObject(object);
            }
            catch (EOFException _) {
               eof = true;
            }
         }
      }
      finally {
         this.inputStream.close();
      }
   }

   public static interface Delegate
   {
      public void didReadObject(Object object) throws Exception;
   }
}
