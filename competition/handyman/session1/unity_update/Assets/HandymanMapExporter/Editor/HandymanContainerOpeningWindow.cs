#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace HandymanMapExporter {
// Read-only vertex inspection: never edits the match scene, meshes or materials.
public class HandymanContainerOpeningWindow : EditorWindow {
    GameObject container;
    string destinationId = "trash_box_for_bottle_can#living_room";
    readonly List<Vector3> selected = new List<Vector3>();
    Vector3[] vertices = new Vector3[0];
    bool pick;
    string error;
    [MenuItem("Tools/Handyman Map Exporter/Export Container Inner Opening")]
    public static void Open() { GetWindow<HandymanContainerOpeningWindow>("Container Opening"); }
    void OnEnable() { SceneView.duringSceneGui += OnScene; }
    void OnDisable() { SceneView.duringSceneGui -= OnScene; }
    void OnGUI() {
        EditorGUILayout.HelpBox("Select the real bin model. Click Read vertices, then enable Pick inner rim. In Scene view, click INNER rim vertices in boundary order. Select at least 4; avoid the outer bounding box. No scene changes are made.", MessageType.Info);
        container=(GameObject)EditorGUILayout.ObjectField("Bin model",container,typeof(GameObject),true);
        destinationId=EditorGUILayout.TextField("Destination ID",destinationId);
        if(GUILayout.Button("Read vertices from selected model")) ReadVertices();
        pick=EditorGUILayout.Toggle("Pick inner rim",pick);
        EditorGUILayout.LabelField("Selected boundary vertices",selected.Count.ToString());
        if(GUILayout.Button("Undo last point") && selected.Count>0) selected.RemoveAt(selected.Count-1);
        if(GUILayout.Button("Clear points")) selected.Clear();
        if(!string.IsNullOrEmpty(error)) EditorGUILayout.HelpBox(error,MessageType.Error);
        using(new EditorGUI.DisabledScope(selected.Count<4 || !container)) {
            if(GUILayout.Button("Export reviewed inner opening JSON")) Export();
        }
    }
    void ReadVertices() {
        error=null;selected.Clear();var all=new List<Vector3>();
        if(!container) { error="Select the actual bin model first.";return; }
        foreach(var f in container.GetComponentsInChildren<MeshFilter>(true)) {
            if(!f.sharedMesh) continue;
            // MeshUtility can read imported mesh vertices in the editor without modifying Read/Write settings.
            try { foreach(var p in f.sharedMesh.vertices) all.Add(f.transform.TransformPoint(p)); }
            catch(Exception e) { error="Cannot read mesh " + f.name + ": " + e.Message;vertices=new Vector3[0];return; }
        }
        vertices=all.Distinct().ToArray();SceneView.RepaintAll();
        if(vertices.Length==0) error="No readable MeshFilter vertices. Select the bin mesh, not DestinationCandidates placeholder.";
    }
    void OnScene(SceneView view) {
        if(!pick || vertices.Length==0) return;
        Event e=Event.current;
        if(e.type==EventType.Layout) HandleUtility.AddDefaultControl(GUIUtility.GetControlID(FocusType.Passive));
        Vector3 nearest=vertices[0];float best=12f;
        bool found=false;
        foreach(var p in vertices) {
            if(view.camera.WorldToViewportPoint(p).z<=0) continue;
            float d=Vector2.Distance(HandleUtility.WorldToGUIPoint(p),e.mousePosition);
            if(d<best) { nearest=p;best=d;found=true; }
        }
        Handles.color=Color.cyan;
        if(found) Handles.SphereHandleCap(0,nearest,Quaternion.identity,HandleUtility.GetHandleSize(nearest)*.025f,EventType.Repaint);
        Handles.color=Color.green;
        for(int i=0;i<selected.Count;i++) {
            Handles.Label(selected[i],i.ToString());
            if(i>0) Handles.DrawLine(selected[i-1],selected[i]);
        }
        if(selected.Count>2) Handles.DrawLine(selected.Last(),selected[0]);
        if(e.type==EventType.MouseDown && e.button==0 && !e.alt && found) {
            if(!selected.Any(p=>Vector3.Distance(p,nearest)<.001f)) selected.Add(nearest);
            e.Use();Repaint();
        }
        view.Repaint();
    }
    string F(float x) { return x.ToString("R",CultureInfo.InvariantCulture); }
    string Q(string x) { return "\""+x.Replace("\\","\\\\").Replace("\"","\\\"")+"\""; }
    void Export() {
        if(string.IsNullOrWhiteSpace(destinationId) || !destinationId.Contains("#")) { error="Use exact destination ID including #room.";return; }
        string file=EditorUtility.SaveFilePanel("Export inner opening", "", "container-inner-opening", "json");
        if(string.IsNullOrEmpty(file)) return;
        // Same ROS map conversion as HandymanMapExporter: (z,-x,y).
        string polygon=string.Join(",",selected.Select(p=>"["+F(p.z)+","+F(-p.x)+"]"));
        string corners=string.Join(",",selected.Select(p=>"["+F(p.z)+","+F(-p.x)+","+F(p.y)+"]"));
        string text="{\n\"destination_id\":"+Q(destinationId)+",\n\"polygon_map_m\":["+polygon+"],\n\"rim_height_m\":"+F(selected.Max(p=>p.y))+",\n\"measured_corners_map_m\":["+corners+"],\n\"source\":\"reviewed_unity_inner_rim_mesh_vertices\",\n\"mesh_model\":"+Q(container.name)+",\n\"coordinate_frame\":\"map_ros_z_minus_x_y\",\n\"autonomous_opening_detection\":false\n}";
        File.WriteAllText(file,text);error=null;Debug.Log("Exported reviewed inner opening: "+file);
    }
}
}
#endif
