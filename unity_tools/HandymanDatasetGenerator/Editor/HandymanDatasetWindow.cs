using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using Object = UnityEngine.Object;

namespace HandymanDatasetGenerator
{
    // Editor-only: source components are never instantiated or enabled.
    public sealed class HandymanDatasetWindow : EditorWindow
    {
        static readonly string[] Names = { "apple", "canned_juice", "rabbit_doll", "pink_cup", "white_cup", "filled_ketchup" };
        static readonly int[] GlobalIds = { 0, 3, 15, 14, 26, 9 };
        [SerializeField] GameObject[] sources = new GameObject[6];
        [SerializeField] string output = "D:/handyman-dataset/pilot03";
        [SerializeField] int count = 50, seed = 20260909, width = 640, height = 480;
        [SerializeField] float fov = 46.82f;
        [SerializeField] Camera referenceCamera;
        [SerializeField] Material floorSource, wallSource;
        [SerializeField] GameObject furnitureSource;
        [SerializeField] bool ketchupCoreMask;
        readonly HashSet<Renderer> coreMaskRenderers = new HashSet<Renderer>();
        readonly HashSet<Renderer> doubleSidedLabelRenderers = new HashSet<Renderer>();
        Vector2 scroll;
        PreviewRenderUtility preview;
        readonly List<Object> owned = new List<Object>();
        readonly List<GameObject> objects = new List<GameObject>();
        readonly List<Quaternion> baseRotations = new List<Quaternion>();
        readonly Dictionary<Renderer, Material[]> rgbMaterials = new Dictionary<Renderer, Material[]>();
        readonly Dictionary<Renderer, Material[]> idMaterials = new Dictionary<Renderer, Material[]>();
        GameObject table;
        GameObject backgroundFurniture;
        Quaternion furnitureBaseRotation;
        Material tableMaterial;
        int next;
        bool running;
        string status = "Open the Handyman scene (NOT Play Mode), then Find Sources.";
        Batch batch;

        [Serializable] public class SourceInfo { public string name, hierarchy, assetGuid, dependencyHash; public int globalClassId; }
        [Serializable] public class Batch
        {
            public int schemaVersion = 1, seed, count, width, height;
            public string generatorVersion = "4.3", sampling = "six classes; fully framed side-by-side pairs; 3x3 single/mixed positions; background-focused views";
            public string transparencyPolicy;
            public string floorAsset, wallAsset, furnitureAsset;
            public float verticalFov;
            public string unityVersion, pipeline, cameraReference;
            public SourceInfo[] sources;
        }
        [Serializable] public class InstanceInfo
        {
            public int instanceId, classId, globalClassId;
            public string className;
            public Vector3 position, euler;
        }
        [Serializable] public class Frame
        {
            public int schemaVersion = 1, index, seed, width, height;
            public string group, rgb, mask;
            public bool negative;
            public float verticalFov;
            public float cameraDistance;
            public string distanceBand;
            public Vector3 cameraPosition, cameraEuler;
            public int anchorClassId, viewportCell;
            public Vector2 requestedViewport;
            public Vector3 actualAnchorViewport;
            public bool backgroundFocused;
            public string objectSampling;
            public string transparencyPolicy;
            public Vector3 furniturePosition, furnitureEuler;
            public InstanceInfo[] instances;
        }

        [MenuItem("Tools/Handyman Dataset Generator")]
        public static void Open() { GetWindow<HandymanDatasetWindow>("Handyman Dataset"); }

        void OnGUI()
        {
            EnsureSourceSlots();
            scroll = EditorGUILayout.BeginScrollView(scroll);
            EditorGUILayout.HelpBox("V4.3: fully framed juice/ketchup pairs; opaque-core + double-sided label. RGB unchanged. Use a NEW folder.", MessageType.Info);
            using (new EditorGUI.DisabledScope(running))
            {
                if (GUILayout.Button("1. Find Missing Sources (keep assigned)")) FindSources();
                for (int i = 0; i < Names.Length; ++i)
                    sources[i] = (GameObject)EditorGUILayout.ObjectField(Names[i], sources[i], typeof(GameObject), true);
                referenceCamera = (Camera)EditorGUILayout.ObjectField("Optional RGB camera", referenceCamera, typeof(Camera), true);
                if (referenceCamera && GUILayout.Button("Copy RGB camera vertical FOV")) fov = referenceCamera.fieldOfView;
                floorSource = (Material)EditorGUILayout.ObjectField("Optional floor material", floorSource, typeof(Material), false);
                wallSource = (Material)EditorGUILayout.ObjectField("Optional wall material", wallSource, typeof(Material), false);
                furnitureSource = (GameObject)EditorGUILayout.ObjectField("Optional background furniture", furnitureSource, typeof(GameObject), true);
                ketchupCoreMask = EditorGUILayout.Toggle("Ketchup opaque-core mask", ketchupCoreMask);
                EditorGUILayout.HelpBox("Opt-in: exclude only verified phong1 slot 2 from mask color/depth, retaining its original RGB. Labels describe opaque core, NOT complete bottle silhouette or grasp boundary. Other transparency remains unsupported.", MessageType.Warning);
                EditorGUILayout.HelpBox("For false-positive training, assign the actual empty trash-bin model (including its printed graphic) as background furniture. Never assign a Layout or furniture containing target objects. Background receives ID 0; materials are copied, never edited.", MessageType.None);
                width = EditorGUILayout.IntField("Width", width);
                height = EditorGUILayout.IntField("Height", height);
                fov = EditorGUILayout.FloatField("Vertical FOV (degrees)", fov);
                count = EditorGUILayout.IntField("Frame count (50 first)", count);
                seed = EditorGUILayout.IntField("Batch seed", seed);
                output = EditorGUILayout.TextField("Output directory", output);
                if (GUILayout.Button("Choose Output Directory"))
                {
                    string selected = EditorUtility.OpenFolderPanel("Dataset output", "", "");
                    if (!string.IsNullOrEmpty(selected)) output = selected;
                }
                if (GUILayout.Button("2. Generate / Resume"))
                {
                    try { StartBatch(); }
                    catch (Exception e) { Stop(); status = e.Message; Debug.LogException(e); }
                }
            }
            if (running && GUILayout.Button("Cancel (completed frames retained)")) { Stop(); status = "Cancelled; use identical settings to resume."; }
            EditorGUILayout.HelpBox(status, MessageType.None);
            EditorGUILayout.EndScrollView();
        }

        static string Hierarchy(Transform t)
        {
            return t.parent ? Hierarchy(t.parent) + "/" + t.name : t.name;
        }

        void FindSources()
        {
            EnsureSourceSlots();
            var all = Resources.FindObjectsOfTypeAll<GameObject>()
                .Where(g => g.scene.IsValid() && g.scene.isLoaded && !EditorSceneManager.IsPreviewScene(g.scene))
                .OrderBy(g => g.scene.path + "/" + Hierarchy(g.transform)).ToArray();
            for (int i = 0; i < Names.Length; ++i)
            {
                if (sources[i]) continue;
                sources[i] = all.Where(g => MatchesClass(g.name, Names[i]) && g.GetComponentsInChildren<MeshRenderer>(true).Any(r => r.enabled))
                    .OrderByDescending(g => g.GetComponent<Rigidbody>() != null)
                    .ThenByDescending(g => Hierarchy(g.transform).Contains("GraspingCandidates/"))
                    .ThenBy(g => Hierarchy(g.transform), StringComparer.Ordinal).FirstOrDefault();
            }
            status = "Assigned " + sources.Count(s => s) + "/" + Names.Length + "; existing choices preserved. Verify actual model roots before capture.";
        }

        void EnsureSourceSlots()
        {
            if (sources == null) sources = new GameObject[Names.Length];
            else if (sources.Length != Names.Length) Array.Resize(ref sources, Names.Length);
        }

        internal static bool MatchesClass(string objectName, string className)
        {
            // Only explicit numeric instance suffixes; never apple_tree or pineapple.
            return Regex.IsMatch(objectName, "^" + Regex.Escape(className) + @"(?:#\d+|_\d+)?(?:\(Clone\))?$");
        }

        static string AssetSignature(Object source)
        {
            if (!source) return "procedural-default";
            string path = AssetDatabase.GetAssetPath(source);
            var go = source as GameObject;
            if (string.IsNullOrEmpty(path) && go) path = go.scene.path;
            return AssetDatabase.AssetPathToGUID(path) + ":" + AssetDatabase.GetAssetDependencyHash(path) + ":" +
                (go ? Hierarchy(go.transform) : source.name);
        }

        void StartBatch()
        {
            EnsureSourceSlots();
            if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Exit Play Mode before capture.");
            if (sources.Any(s => !s)) throw new InvalidOperationException("All six source models must be assigned (including filled_ketchup).");
            if (sources.Any(s => s.scene.IsValid() && s.scene.isDirty))
                throw new InvalidOperationException("Source scene has unsaved changes. Save or revert them yourself before capture; the tool never saves your scene.");
            if (furnitureSource && furnitureSource.scene.IsValid() && furnitureSource.scene.isDirty)
                throw new InvalidOperationException("Background source scene has unsaved changes.");
            if (width < 64 || height < 64 || width > 2048 || height > 2048 || count < 1 || count > 10000 || fov < 20 || fov > 100)
                throw new InvalidOperationException("Use resolution 64..2048, frames 1..10000, vertical FOV 20..100.");
            if (!Path.IsPathRooted(output)) throw new InvalidOperationException("Output must be an absolute path outside Assets.");
            output = Path.GetFullPath(output);
            string assets = Path.GetFullPath(Application.dataPath).TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar;
            if ((output + Path.DirectorySeparatorChar).StartsWith(assets, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException("Choose a directory outside Assets to avoid repeated imports.");
            if (!(GraphicsSettings.currentRenderPipeline is UniversalRenderPipelineAsset))
                throw new InvalidOperationException("This version targets the inspected Unity 6 URP project.");
            batch = new Batch { seed = seed, count = count, width = width, height = height, verticalFov = fov,
                transparencyPolicy = ketchupCoreMask ? "filled-ketchup-opaque-core-v1" : "opaque-only",
                floorAsset = AssetSignature(floorSource), wallAsset = AssetSignature(wallSource), furnitureAsset = AssetSignature(furnitureSource),
                unityVersion = Application.unityVersion, pipeline = GraphicsSettings.currentRenderPipeline.name,
                cameraReference = referenceCamera ? Hierarchy(referenceCamera.transform) : "manual-unverified",
                sources = sources.Select((s, i) => {
                    string path = AssetDatabase.GetAssetPath(s);
                    if (string.IsNullOrEmpty(path)) path = s.scene.path;
                    return new SourceInfo { name = Names[i], globalClassId = GlobalIds[i], hierarchy = Hierarchy(s.transform),
                        assetGuid = AssetDatabase.AssetPathToGUID(path), dependencyHash = AssetDatabase.GetAssetDependencyHash(path).ToString() };
                }).ToArray() };
            string manifest = JsonUtility.ToJson(batch, true);
            string manifestPath = Path.Combine(output, "batch.json");
            if (Directory.Exists(output) && Directory.EnumerateFileSystemEntries(output).Any())
            {
                if (!File.Exists(manifestPath) || File.ReadAllText(manifestPath) != manifest)
                    throw new InvalidOperationException("Directory is nonempty or settings/assets changed. Choose a NEW batch directory; nothing was overwritten.");
            }
            Shader idShader = Shader.Find("Hidden/HandymanDataset/InstanceId");
            if (!idShader || !idShader.isSupported) throw new InvalidOperationException("InstanceId shader missing/unsupported. Copy the complete tool folder.");
            preview = new PreviewRenderUtility();
            preview.camera.clearFlags = CameraClearFlags.SolidColor;
            preview.camera.nearClipPlane = .02f;
            preview.camera.farClipPlane = 20;
            preview.camera.allowHDR = false;
            preview.camera.allowMSAA = false;
            preview.camera.fieldOfView = fov;
            preview.camera.aspect = (float)width / height;
            var data = preview.camera.GetUniversalAdditionalCameraData();
            data.renderPostProcessing = false;
            data.antialiasing = AntialiasingMode.None;
            data.volumeLayerMask = 0;
            preview.ambientColor = new Color(.35f, .35f, .35f);
            for (int i = 0; i < Names.Length; ++i)
            {
                var root = CopyGeometry(sources[i].transform, null, true);
                root.name = Names[i];
                preview.AddSingleGO(root);
                objects.Add(root);
                baseRotations.Add(root.transform.rotation);
                Bounds b = BoundsOf(root);
                if (b.size.magnitude < .01f || b.size.magnitude > 1.5f)
                    throw new InvalidOperationException(Names[i] + " has unexpected physical size: " + b.size);
                Register(root, i + 1, idShader);
            }
            table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Object.DestroyImmediate(table.GetComponent<Collider>());
            preview.AddSingleGO(table);
            table.transform.position = new Vector3(0, -.06f, 0);
            table.transform.localScale = new Vector3(1.8f, .12f, 1.2f);
            tableMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            owned.Add(tableMaterial);
            table.GetComponent<Renderer>().sharedMaterial = tableMaterial;
            Register(table, 0, idShader); // Background still writes depth: it occludes labels correctly.
            AddBackdrop("Floor", new Vector3(0, -.86f, 0), new Vector3(6, .12f, 6), floorSource, new Color(.65f, .48f, .3f), idShader);
            AddBackdrop("Back wall", new Vector3(0, .7f, 1.9f), new Vector3(6, 3, .12f), wallSource, new Color(.72f, .76f, .8f), idShader);
            if (furnitureSource)
            {
                if (furnitureSource.GetComponentsInChildren<Transform>(true).Any(t => Names.Any(n => MatchesClass(t.name, n))))
                    throw new InvalidOperationException("Background contains a target object; remove it from the source selection to avoid missing labels.");
                var furniture = CopyGeometry(furnitureSource.transform, null, true);
                backgroundFurniture = furniture;
                furnitureBaseRotation = furniture.transform.rotation;
                preview.AddSingleGO(furniture);
                Bounds b = BoundsOf(furniture);
                if (b.size.x > 1.5f || b.size.z > .8f || b.size.y > 2.2f)
                    throw new InvalidOperationException("Choose small furniture (width <=1.5m, depth <=0.8m, height <=2.2m), not a room root.");
                furniture.transform.position = new Vector3(.65f, -.8f + b.extents.y, 1.3f) - b.center;
                Register(furniture, 0, idShader);
            }
            Directory.CreateDirectory(output);
            foreach (string dir in new[] { "rgb", "instance", "frames" }) Directory.CreateDirectory(Path.Combine(output, dir));
            if (!File.Exists(manifestPath)) File.WriteAllText(manifestPath, manifest);
            next = 0;
            running = true;
            EditorApplication.update += Tick;
            status = "Capturing one frame per editor update...";
        }

        void AddBackdrop(string name, Vector3 position, Vector3 scale, Material original, Color color, Shader idShader)
        {
            if (original && (original.renderQueue >= 2450 || original.IsKeywordEnabled("_ALPHATEST_ON") ||
                (original.HasProperty("_Surface") && original.GetFloat("_Surface") != 0)))
                throw new InvalidOperationException("Backdrop material must be opaque.");
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            preview.AddSingleGO(go);
            go.name = name;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.transform.position = position;
            go.transform.localScale = scale;
            var material = original ? new Material(original) : new Material(Shader.Find("Universal Render Pipeline/Lit"));
            owned.Add(material);
            if (!original) material.SetColor("_BaseColor", color);
            go.GetComponent<Renderer>().sharedMaterial = material;
            Register(go, 0, idShader);
        }

        GameObject CopyGeometry(Transform source, Transform parent, bool root)
        {
            var go = new GameObject(source.name) { hideFlags = HideFlags.HideAndDontSave };
            // Track even a partially constructed tree, so validation failures cannot leak objects.
            owned.Add(go);
            if (parent) go.transform.SetParent(parent, false);
            go.transform.localPosition = root ? Vector3.zero : source.localPosition;
            go.transform.localRotation = root ? source.rotation : source.localRotation;
            go.transform.localScale = root ? source.lossyScale : source.localScale;
            if (source.GetComponent<SkinnedMeshRenderer>())
                throw new InvalidOperationException("Skinned mesh not supported in pilot: " + Hierarchy(source));
            var renderer = source.GetComponent<MeshRenderer>();
            if (renderer && renderer.enabled)
            {
                var filter = source.GetComponent<MeshFilter>();
                if (!filter || !filter.sharedMesh) throw new InvalidOperationException("Missing mesh: " + source.name);
                bool core = ketchupCoreMask && MatchesClass(source.name, "filled_ketchup") &&
                    renderer.sharedMaterials.Length == 4 && filter.sharedMesh.subMeshCount == 4;
                for (int slot = 0; slot < renderer.sharedMaterials.Length; slot++)
                {
                    var mat = renderer.sharedMaterials[slot];
                    if (core && slot == 2 && VerifiedKetchupShell(mat)) continue;
                    if (core && slot == 3 && VerifiedKetchupLabel(mat)) continue;
                    if (!mat || mat.renderQueue >= 2450 || mat.IsKeywordEnabled("_ALPHATEST_ON") ||
                        (mat.HasProperty("_Surface") && mat.GetFloat("_Surface") != 0) ||
                        (mat.HasProperty("_Cull") && mat.GetFloat("_Cull") != 2))
                        throw new InvalidOperationException("Unsupported material: " + Hierarchy(source) + " slot=" + slot + " material=" + (mat ? mat.name : "null") + ". Do not silently label it as opaque.");
                }
                if (renderer.HasPropertyBlock()) throw new InvalidOperationException("MaterialPropertyBlock requires explicit support: " + source.name);
                go.AddComponent<MeshFilter>().sharedMesh = filter.sharedMesh;
                var cloneRenderer = go.AddComponent<MeshRenderer>();
                cloneRenderer.sharedMaterials = renderer.sharedMaterials;
                if (core && VerifiedKetchupLabel(renderer.sharedMaterials[3]))
                    doubleSidedLabelRenderers.Add(cloneRenderer);
                if (core && VerifiedKetchupShell(renderer.sharedMaterials[2]))
                {
                    coreMaskRenderers.Add(cloneRenderer);
                    Debug.Log("Ketchup mask audit: " + Hierarchy(source) + " mesh=" + filter.sharedMesh.name +
                        " submesh=2 indices=" + filter.sharedMesh.GetIndexCount(2) +
                        " material=" + renderer.sharedMaterials[2].name + " policy=opaque-core (RGB unchanged)");
                }
            }
            foreach (Transform child in source)
                if (child.gameObject.activeSelf) CopyGeometry(child, go.transform, false);
            return go;
        }

        void Register(GameObject root, int id, Shader shader)
        {
            var mat = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            mat.SetColor("_IdColor", new Color((id & 1) != 0 ? 1 : 0, (id & 2) != 0 ? 1 : 0, (id & 4) != 0 ? 1 : 0, 1));
            owned.Add(mat);
            foreach (var r in root.GetComponentsInChildren<Renderer>())
            {
                rgbMaterials.Add(r, r.sharedMaterials);
                var slots = Enumerable.Repeat(mat, r.sharedMaterials.Length).ToArray();
                if (doubleSidedLabelRenderers.Contains(r))
                {
                    if (id != 6) throw new InvalidOperationException("Ketchup label requires instance 6");
                    if (!mat.HasProperty("_Cull")) throw new InvalidOperationException("Update InstanceId.shader with V4.2");
                    var label = new Material(mat);
                    label.SetFloat("_Cull", 0);
                    label.SetFloat("_MaskEnabled", 1);
                    owned.Add(label);
                    slots[3] = label;
                }
                if (coreMaskRenderers.Contains(r))
                {
                    if (id != 6) throw new InvalidOperationException("Core mask allowed only for filled_ketchup instance 6");
                    var skip = new Material(mat);
                    skip.SetFloat("_MaskEnabled", 0);
                    owned.Add(skip);
                    slots[2] = skip;
                }
                idMaterials.Add(r, slots);
            }
        }

        internal static bool VerifiedKetchupShell(Material mat)
        {
            return mat && mat.name == "phong1" && mat.shader && mat.shader.name == "Universal Render Pipeline/Lit" &&
                AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(mat)) == "c2c237c9018062a43b32d3597d2b1b25" &&
                mat.GetFloat("_Surface") == 1 && mat.GetFloat("_Cull") == 0 && mat.GetFloat("_Blend") == 0 &&
                mat.GetFloat("_AlphaClip") == 0 && !mat.IsKeywordEnabled("_ALPHATEST_ON") &&
                mat.GetTexture("_BaseMap") == null && Mathf.Abs(mat.GetColor("_BaseColor").a - 100f / 255) < .005f;
        }

        internal static bool VerifiedKetchupLabel(Material mat)
        {
            return mat && mat.name == "ketchup_label" && mat.shader && mat.shader.name == "Universal Render Pipeline/Lit" &&
                AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(mat)) == "343e0848cdc851c4fbd32055d297f791" &&
                mat.renderQueue < 2450 && mat.GetFloat("_Surface") == 0 && mat.GetFloat("_Cull") == 0 &&
                mat.GetFloat("_AlphaClip") == 0 && !mat.IsKeywordEnabled("_ALPHATEST_ON");
        }

        static Bounds BoundsOf(GameObject root)
        {
            var rs = root.GetComponentsInChildren<Renderer>();
            if (rs.Length == 0) throw new InvalidOperationException("No visible renderers under " + root.name);
            Bounds b = rs[0].bounds;
            foreach (var r in rs.Skip(1)) b.Encapsulate(r.bounds);
            return b;
        }

        static float Range(System.Random r, float min, float max) { return min + (float)r.NextDouble() * (max - min); }

        // Viewport coordinates use Unity's bottom-left origin. Random cells avoid
        // coupling image position to class (index % 5) or distance (index % 3).
        internal static Vector2 SampleViewport(System.Random random, out int cell)
        {
            cell = random.Next(9);
            return new Vector2(.12f + (cell % 3) * .38f + Range(random, -.04f, .04f),
                .18f + (cell / 3) * .32f + Range(random, -.04f, .04f));
        }

        internal static Quaternion ViewportRotation(Vector2 viewport, float verticalFov, float aspect)
        {
            float tangent = Mathf.Tan(verticalFov * Mathf.Deg2Rad * .5f);
            var ray = new Vector3((2 * viewport.x - 1) * tangent * aspect,
                (2 * viewport.y - 1) * tangent, 1).normalized;
            return Quaternion.FromToRotation(ray, Vector3.forward);
        }

        void Tick()
        {
            if (!running) return;
            if (EditorApplication.isPlayingOrWillChangePlaymode || EditorApplication.isCompiling) { Stop(); return; }
            try
            {
                if (next >= count) { Stop(); status = "Complete: " + output + ". Validate RGB/mask alignment before training."; Repaint(); return; }
                string stem = next.ToString("D6", CultureInfo.InvariantCulture);
                string framePath = Path.Combine(output, "frames", stem + ".json");
                if (File.Exists(framePath))
                {
                    if (!File.Exists(Path.Combine(output, "rgb", stem + ".png")) || !File.Exists(Path.Combine(output, "instance", stem + ".png")))
                        throw new InvalidOperationException("Committed frame missing image: " + stem + ". Use a new batch directory.");
                    next++; return;
                }
                Capture(stem, framePath);
                next++;
                status = "Captured " + next + "/" + count + " | " + output;
                Repaint();
            }
            catch (Exception e) { Stop(); status = "Stopped: " + e.Message; Debug.LogException(e); Repaint(); }
        }

        void Capture(string stem, string framePath)
        {
            var random = new System.Random(unchecked(seed + next * 7919));
            bool backgroundFocused = backgroundFurniture && next % 3 == 2;
            bool negative = backgroundFocused || next % 10 == 9;
            if (backgroundFurniture)
            {
                backgroundFurniture.transform.position = Vector3.zero;
                float yaw = (backgroundFocused ? (next / 3) % 8 * 45f : Range(random, 0, 360)) + Range(random, -10, 10);
                backgroundFurniture.transform.rotation = Quaternion.Euler(0, yaw, 0) * furnitureBaseRotation;
                Bounds fb = BoundsOf(backgroundFurniture);
                // Entire bounds clear the table on both x and z. Keep the base on
                // the preview floor (y=-.8); never move the source scene object.
                float side = (next / 3) % 2 == 0 ? -1 : 1;
                Vector3 center = new Vector3(side * (.9f + fb.extents.x + .2f),
                    -.8f + fb.extents.y, -.6f - fb.extents.z - .25f);
                backgroundFurniture.transform.position = center - fb.center;
            }
            // Do not use next % 6: every-third background frames would permanently
            // exclude two anchor classes, including ketchup.
            int anchor = ((next / 3) * 2 + next % 3) % Names.Length;
            int objectMode = (next / 9) % 3;
            if (objectMode == 1) anchor = (next / 3) % 2 == 0 ? 1 : 5;
            var instances = new List<InstanceInfo>();
            var placed = new List<Bounds>();
            for (int offset = 0; offset < Names.Length; ++offset)
            {
                int i = (anchor + offset) % Names.Length;
                var root = objects[i];
                bool active = !negative && (offset == 0 ||
                    (objectMode == 1 ? (i == 1 || i == 5) : objectMode == 2 && random.NextDouble() < .5));
                root.SetActive(active);
                if (!active) continue;
                root.transform.position = Vector3.zero;
                root.transform.rotation = Quaternion.Euler(0, Range(random, 0, 360), 0) * baseRotations[i];
                Bounds b = BoundsOf(root);
                bool valid = false;
                for (int trial = 0; trial < 100; ++trial)
                {
                    Vector3 center = new Vector3(offset == 0 ? 0 : Range(random, -.6f, .6f), b.extents.y, offset == 0 ? 0 : Range(random, -.45f, .45f));
                    if (objectMode == 1)
                        center = new Vector3((i == 1 ? -1 : 1) * (next % 2 == 0 ? 1 : -1) * (.12f + b.extents.x), b.extents.y, 0);
                    Bounds proposed = new Bounds(center, b.size + new Vector3(.025f, 0, .025f));
                    if (proposed.min.x < -.9f || proposed.max.x > .9f || proposed.min.z < -.6f || proposed.max.z > .6f) continue;
                    if (placed.Any(p => p.Intersects(proposed))) continue;
                    root.transform.position = center - b.center;
                    placed.Add(proposed); valid = true; break;
                }
                if (!valid) { root.SetActive(false); continue; }
                instances.Add(new InstanceInfo { instanceId = i + 1, classId = i, globalClassId = GlobalIds[i], className = Names[i],
                    position = root.transform.position, euler = root.transform.eulerAngles });
            }
            int band = (next / 3 + next % 3) % 3;
            float distance = band == 0 ? Range(random, .55f, .85f) : band == 1 ? Range(random, .85f, 1.3f) : Range(random, 1.3f, 2f);
            float azimuth = Range(random, -55, 55), elevation = Range(random, 18, 50);
            Vector3 target = objects[anchor].activeSelf ? BoundsOf(objects[anchor]).center : new Vector3(0, .12f, 0);
            bool pairFocused = !negative && objectMode == 1;
            if (pairFocused)
            {
                if (!objects[1].activeSelf || !objects[5].activeSelf)
                    throw new InvalidOperationException("Pair placement failed; no partial pair frame will be committed.");
                Bounds combined = BoundsOf(objects[1]);
                combined.Encapsulate(BoundsOf(objects[5]));
                target = combined.center;
                float halfFov = Mathf.Atan(Mathf.Tan(fov * Mathf.Deg2Rad * .5f) * Mathf.Min(1, (float)width / height));
                distance = Mathf.Max(.7f, combined.extents.magnitude / Mathf.Sin(halfFov) * 1.35f);
                azimuth = 0; // Side-by-side separation in the image; object yaw remains random.
            }
            if (backgroundFocused)
            {
                Bounds fb = BoundsOf(backgroundFurniture);
                target = fb.center;
                // Bounding sphere fits even at the narrowest FOV axis; dedicated
                // views expose the bin body without raising it off the floor.
                float halfFov = Mathf.Atan(Mathf.Tan(fov * Mathf.Deg2Rad * .5f) * Mathf.Min(1, (float)width / height));
                distance = Mathf.Max(1, fb.extents.magnitude / Mathf.Sin(halfFov) * 1.15f);
                azimuth = Range(random, -15, 15);
                elevation = Range(random, 8, 20);
            }
            preview.camera.transform.position = target + Quaternion.Euler(elevation, azimuth, 0) * new Vector3(0, 0, -distance);
            preview.camera.transform.LookAt(target);
            int viewportCell;
            Vector2 requestedViewport = SampleViewport(random, out viewportCell);
            if (backgroundFocused || pairFocused) { requestedViewport = new Vector2(.5f, .5f); viewportCell = 4; }
            preview.camera.transform.rotation *= ViewportRotation(requestedViewport, fov, (float)width / height);
            if (pairFocused)
                foreach (int id in new[] { 1, 5 })
                {
                    Bounds bounds = BoundsOf(objects[id]);
                    for (int corner = 0; corner < 8; corner++)
                    {
                        Vector3 point = bounds.center + Vector3.Scale(bounds.extents,
                            new Vector3((corner & 1) == 0 ? -1 : 1, (corner & 2) == 0 ? -1 : 1, (corner & 4) == 0 ? -1 : 1));
                        Vector3 uv = preview.camera.WorldToViewportPoint(point);
                        if (uv.z <= preview.camera.nearClipPlane || uv.z >= preview.camera.farClipPlane ||
                            uv.x < .05f || uv.x > .95f || uv.y < .05f || uv.y > .95f)
                            throw new InvalidOperationException("Pair framing check failed; no clipped pair frame will be committed.");
                    }
                }
            Vector3 actualAnchorViewport = preview.camera.WorldToViewportPoint(target);
            preview.lights[0].transform.rotation = Quaternion.Euler(Range(random, 25, 75), Range(random, 0, 360), 0);
            preview.lights[0].intensity = Range(random, .7f, 1.4f);
            preview.lights[1].intensity = Range(random, .2f, .7f);
            tableMaterial.SetColor("_BaseColor", Color.HSVToRGB(Range(random, 0, 1), Range(random, 0, .3f), Range(random, .25f, .8f)));
            foreach (var pair in rgbMaterials) pair.Key.sharedMaterials = pair.Value;
            preview.camera.backgroundColor = new Color(.22f, .25f, .28f);
            byte[] rgb = RenderPng(false);
            foreach (var pair in idMaterials) pair.Key.sharedMaterials = pair.Value;
            preview.camera.backgroundColor = Color.black;
            byte[] mask;
            try { mask = RenderPng(true); }
            finally { foreach (var pair in rgbMaterials) pair.Key.sharedMaterials = pair.Value; }
            var frame = new Frame { index = next, seed = seed, width = width, height = height, group = "seed_" + seed,
                cameraDistance = distance, distanceBand = backgroundFocused ? "background-fit" : pairFocused ? "pair-fit" : new[] { "near", "mid", "far" }[band],
                negative = negative, verticalFov = fov, rgb = "rgb/" + stem + ".png", mask = "instance/" + stem + ".png",
                anchorClassId = negative ? -1 : anchor, viewportCell = viewportCell,
                requestedViewport = requestedViewport, actualAnchorViewport = actualAnchorViewport,
                backgroundFocused = backgroundFocused,
                transparencyPolicy = batch.transparencyPolicy,
                objectSampling = negative ? "background" : new[] { "single", "juice-ketchup-pair", "mixed" }[objectMode],
                furniturePosition = backgroundFurniture ? backgroundFurniture.transform.position : Vector3.zero,
                furnitureEuler = backgroundFurniture ? backgroundFurniture.transform.eulerAngles : Vector3.zero,
                cameraPosition = preview.camera.transform.position, cameraEuler = preview.camera.transform.eulerAngles, instances = instances.ToArray() };
            // Frame JSON is the commit marker. Incomplete image pairs may be regenerated on resume.
            File.WriteAllBytes(Path.Combine(output, frame.rgb), rgb);
            File.WriteAllBytes(Path.Combine(output, frame.mask), mask);
            File.WriteAllText(framePath, JsonUtility.ToJson(frame, true));
        }

        byte[] RenderPng(bool mask)
        {
            float dpi = EditorGUIUtility.pixelsPerPoint;
            preview.BeginPreview(new Rect(0, 0, width / dpi, height / dpi), GUIStyle.none);
            Texture2D image = null;
            try
            {
                preview.Render(true, false);
                RenderTexture rt = preview.camera.targetTexture;
                if (rt.width != width || rt.height != height) throw new InvalidOperationException("Unexpected preview resolution; check display DPI.");
                RenderTexture.active = rt;
                image = new Texture2D(width, height, TextureFormat.RGB24, false, true);
                image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                image.Apply();
                // Preview RT stores linear color. Encode RGB to sRGB; binary ID endpoints remain unchanged.
                if (!mask && QualitySettings.activeColorSpace == ColorSpace.Linear)
                {
                    var pixels = image.GetPixels();
                    for (int i = 0; i < pixels.Length; ++i) pixels[i] = pixels[i].gamma;
                    image.SetPixels(pixels); image.Apply();
                }
                return image.EncodeToPNG();
            }
            finally { if (image) Object.DestroyImmediate(image); preview.EndPreview(); }
        }

        void Stop()
        {
            running = false;
            EditorApplication.update -= Tick;
            if (preview != null) { preview.Cleanup(); preview = null; }
            foreach (var item in owned) if (item) Object.DestroyImmediate(item);
            owned.Clear(); objects.Clear(); baseRotations.Clear(); rgbMaterials.Clear(); idMaterials.Clear();
            backgroundFurniture = null;
            coreMaskRenderers.Clear();
            doubleSidedLabelRenderers.Clear();
        }
        void OnDisable() { Stop(); }
    }
}
