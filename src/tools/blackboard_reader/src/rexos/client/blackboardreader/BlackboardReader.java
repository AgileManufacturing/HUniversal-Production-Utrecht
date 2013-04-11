/**
 * @author Ammar Abdulamir
 * @file BlackboardReader.java
 * @brief A blackboard reader GUI.
 * @date Created: 4/9/13
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package rexos.client.blackboardreader;

import com.mongodb.BasicDBObject;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import rexos.client.blackboardreader.models.Blackboards;
import rexos.client.blackboardreader.models.Entries;
import rexos.client.blackboardreader.models.EntryNode;

import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.util.ArrayList;

public class BlackboardReader implements ListSelectionListener {
    private JList collections;
    private JPanel mainPanel;
    private JTree entries;
    private final ArrayList<EntryNode> entriesList;
    private final Blackboards blackboards;

    public BlackboardReader() {
        JFrame frame = new JFrame("Blackboard Reader");
        frame.setContentPane(mainPanel);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);

        entriesList = new ArrayList<EntryNode>();
        blackboards = new Blackboards();
        collections.addListSelectionListener(this);

    }

    public Blackboards getBlackboards() {
        collections.updateUI();
        return blackboards;
    }

    public void setListEntries(DBCollection collection) {
        Entries model = new Entries();

        DBCursor cursor = collection.find();
        while (cursor.hasNext()) {
            model.add(new EntryNode((BasicDBObject) cursor.next()));
        }

        entries.setModel(model);
    }

    public void setBlackboards() {
        collections.setModel(blackboards);
    }

    public ArrayList<EntryNode> getEntriesList() {
        return entriesList;
    }

    @Override
    public void valueChanged(ListSelectionEvent e) {
        if (!e.getValueIsAdjusting()) {
            setListEntries((DBCollection) collections.getModel().getElementAt(collections.getSelectedIndex()));
        }
    }
}