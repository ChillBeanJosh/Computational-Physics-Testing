using UnityEngine;
using UnityEngine.SceneManagement;

public class sceneManager : MonoBehaviour
{
    public static sceneManager Instance;

    private void Awake()
    {
        Instance = this;
    }

    //List of Scenes in the build Index, make sure they are in ORDER.
    public enum Scene
    {
        MainMenu,
        FluidTest

    }

    //List of Scene Functions.

    //Used to refrence any specific scene on the build index.
    public void LoadScene(Scene scene)
    {
        SceneManager.LoadScene(scene.ToString());
    }


    //Used for the Start Game Button.
    public void LoadNewGame()
    {
        SceneManager.LoadScene(Scene.FluidTest.ToString());
    }


    //Loads next Scene in build index.
    public void LoadNextScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex + 1);
    }


    //Loads the Main Menu Scene.
    public void LoadMainMenu()
    {
        SceneManager.LoadScene(Scene.MainMenu.ToString());
    }

    //Restarts the current Scene.
    public void RestartScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
}
