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

public class SendCommandTask extends AsyncTask<URL, Void, Void> {

    @Override
    protected Void doInBackground(URL... urls) {
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
                Log.v("Getter", "Your data: " + builder.toString()); //response data
            } else {
                Log.e("Getter", "Failed to download file");
            }
        } catch (ClientProtocolException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    protected void onProgressUpdate(Integer... progress) {
        //setProgressPercent(progress[0]);
        Log.v("debug", "progress called");
    }

    protected void onPostExecute(Long result) {
        //showDialog("Downloaded " + result + " bytes");
        Log.v("debug", "onPostExecute called");
    }


}