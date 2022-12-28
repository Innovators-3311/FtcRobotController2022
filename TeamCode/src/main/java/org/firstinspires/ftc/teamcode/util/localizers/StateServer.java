package org.firstinspires.ftc.teamcode.util.localizers;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class StateServer extends NanoHTTPD {
    public JSONArray response;
    private static final int MAX_STATES = 10000;
    public static int port = 8079;
    private boolean locked = false;

    /**
     * Constructs an HTTP server on given port.
     *
     * @param
     */
    public StateServer() throws IOException {
        super(port);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT,false);
        resetState();
    }

    public void resetState(){
        response = new JSONArray();
//        try
//        {
//            response.put(new JSONObject().put("new", true));
//        } catch (JSONException e) {
//            RobotLog.ee("Localizer", "Error adding marker.");
//        };
    }

    public String getResponse(){
        locked=true;
        String retval = response.toString();
        resetState();
        locked=false;
        return retval;
    }


    public void addState(JSONObject obj){
        if (!locked){
            // Append this state to the end.
            response.put(obj);

            // If there's over 1000 states, remove the first one.
            if (response.length() > MAX_STATES) {
                response.remove(0);
            }
        }
    }

    /**
     * Override this to customize the server.
     * <p>
     * (By default, this returns a 404 "Not Found" plain text error response.)
     *
     * @param session The HTTP session
     * @return HTTP response, see class Response for details
     */
    @Override
    public Response serve(IHTTPSession session) {
        return newFixedLengthResponse(Response.Status.OK, "application/json", getResponse());
    }
}
