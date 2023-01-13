package org.firstinspires.ftc.teamcode.util.localizers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class StateServer extends NanoHTTPD {
    public JSONArray stateLog;
    public boolean valid = true;
    private static final int MAX_STATES = 1000;
    public static int port = 8079;
    private boolean locked = false;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double timeStep = .1;

    private static final StateServer instance = new StateServer();

    /**
     * Constructs an HTTP server on given port.
     *
     * @param
     */
    public StateServer() {
        super(port);
        try {
            start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
            resetState();
        } catch(IOException e){
            valid = false;
        }
    }

    /**
     * Only ever return a single State Server object.
     *
     * @return stateserver.
     */
    public static StateServer getInstance() {
        return instance;
    }

    public void resetState(){
        stateLog = new JSONArray();
    }

    public String getStateLog(){
        locked=true;
        String retval = stateLog.toString();
        resetState();
        locked=false;
        return retval;
    }


    public void addState(JSONObject obj){
        if (!locked && (runtime.seconds() > timeStep)){
            // Append this state to the end.
            stateLog.put(obj);

            // If there's over 1000 states, remove the first one.
            if (stateLog.length() > MAX_STATES) {
                stateLog.remove(0);
            }
            runtime.reset();
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
        return newFixedLengthResponse(Response.Status.OK, "application/json", getStateLog());
    }
}
