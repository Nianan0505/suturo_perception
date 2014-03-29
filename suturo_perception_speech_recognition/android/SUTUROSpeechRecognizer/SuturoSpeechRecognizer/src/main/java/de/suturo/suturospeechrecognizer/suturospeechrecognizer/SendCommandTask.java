package de.suturo.suturospeechrecognizer.suturospeechrecognizer;

import android.os.AsyncTask;
import android.util.Log;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.StatusLine;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;

/**
 * Created by andre on 28.03.14.
 */

/**
 * Async Task to send the text recognized by the speech recognition via http
 * to the ros node that takes care of the parsing and publishing into the ros network
 */
public class SendCommandTask extends AsyncTask<URL, Void, Boolean> {
    private OnTaskCompleted listener;

    public SendCommandTask(OnTaskCompleted listener){
        this.listener = listener;
    }

    @Override
    protected Boolean doInBackground(URL... urls) {
        HttpClient client = new DefaultHttpClient();
        StringBuilder builder = new StringBuilder();
        HttpGet httpGet = new HttpGet(String.valueOf(urls[0]));
        try {
            HttpResponse response = client.execute(httpGet);
            StatusLine statusLine = response.getStatusLine();
            int statusCode = statusLine.getStatusCode();
            if (statusCode == 200) {
                HttpEntity entity = response.getEntity();
                InputStream content = entity.getContent();
                BufferedReader reader = new BufferedReader(
                        new InputStreamReader(content));
                String line;
                while ((line = reader.readLine()) != null) {
                    builder.append(line);
                }

                if (builder.toString().contains("NOK")) {
                    Log.v("HTTP", "OK: Text recognized" + builder.toString()); //response data
                    return Boolean.TRUE;
                }
                else {
                    Log.v("HTTP", "NOK: Text not recognized  " + builder.toString()); //response data
                    return Boolean.FALSE;
                }
            } else {
                Log.e("HTTP", "Failed to download file");
            }
        } catch (ClientProtocolException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return Boolean.FALSE;
    }

    protected void onPostExecute(Boolean result) {
        listener.onTaskCompleted(result);
    }


}