using UnityEngine;

public class SPHSimulator2D : MonoBehaviour
{


    // ======================================================================
    // CALCULATIONS
    // ======================================================================
    //Length, Width, Spacing For Partical Generation:
    void GenerateParticleGrid()
    {

    }

    //Using Smoothing Radius to Calculate the Volume Given Other Particles Within Radius:
   void ComputeParticalVolume()
   {

   }

    //Use Computed Volume to Calculate Density, mass = constant:
    void ComputeParticalDensity()
    {

    }

    //Use Density: [p_i = k * (density_i - restDensity)]
    // p_i = pressure
    // k = GasConstant
    //NOTE: as K increases, time step decreases!
    //DO NOT ALLOW ATTRACTION -> [p_i = max(k*(density_i - restDensity), 0)
    void ComputeParticalPressure()
    {

    }

    //Pressure Force = Summation { (m_j / p_j) * A_j * W_(i*j)
    //ALSO -> -(gradient) * pressure
    //NOTE WE NEED TO ADJUST TO MAKE SYMMETRIC:
    // -> F = - Summation { (m_j / p_j) * (p_i + p_j)/2 * gradient * W_(i*j)
    void ComputePressureForce()
    {

    }

    //Viscosity Force = angularFrequency * Summation { (m_j / p_j) * u_j * (gradient)^2 * W_(i*j)
    //NOTE WE NEED TO ADJUST TO MAKE SYMMETRIC:
    // -> F = angularFrequency * Summation { (m_j / p_j) * u_j - u_i * (gradient)^2 * W_(i*j)
    // W = KernelList.Spiky()
    // Viscosity is ALWAYS NEEDED TO -> Stabilize Particle System!!
    void ComputeViscosityForce()
    {

    }


    //Gravity Force = density_i * gravityValue
    //NOTE: Other Forces can be included (collision, boundary, userinteraction for testing)
    //
    void ComputeGravitationalForce()
    {

    }

    //Force = PressureForce + ViscosityForce + GravityForce
    //Acceleration = Force / Density
    void ParticleTotalForceDensity()
    {

    }

    //Apply TimeSteps:
    // - velocity = u_i(t + 1) = u_i(t) + (deltaT) * Force_i/Density_i
    // - position = x_i(t + 1) = x_i(t) + (deltaT) * u_i(t + 1)
    void Integrate()
    {

    }

    // ======================================================================
    // BOUNDARY
    // ======================================================================

    //Compute a Boundary Box To Hold Particles Inside:
    //if x.y < 0 --> x.y = 0
    void CreateBondaryConditions(float length, float width, float spacing)
    {

    }





}


