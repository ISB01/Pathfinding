using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public class Pathfinding : MonoBehaviour {

	public Transform seekertargetAstar, targetAstar,seekertargetUCS, targetUCS,seekertargetBFS, targetBFS, seekertargetDFS, targetDFS;

	//logging time
	public Stopwatch timerAstar = new Stopwatch();
	public Stopwatch timerBFS = new Stopwatch();
	public Stopwatch timerDFS = new Stopwatch();
	public Stopwatch timerUCS = new Stopwatch();

	//logging expanded nodes
	public int totalMemoryUsedAstar = 0,  totalMemoryUsedUCS = 0, totalMemoryUsedBFS = 0, totalMemoryUsedDFS = 0;

	Grid grid;

	void Awake() {
		grid = GetComponent<Grid> ();
	}

	void Update() { 
	 /*
		FindPath (seekertargetAstar.position, targetAstar.position);
		Debug.Log("A* ===> Time -> "+timerAstar.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedAstar+" (Memory cost -> "+totalMemoryUsedAstar*8+"B for 32b systems and "+totalMemoryUsedAstar*16+"B for 64b systems)");
		FindPathBFS (seekertargetBFS.position, targetBFS.position);
		Debug.Log("BFS ===> Time -> "+timerBFS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedBFS+" (Memory cost -> "+totalMemoryUsedBFS*8+"B for 32b systems and "+totalMemoryUsedBFS*16+"B for 64b systems)");
		FindPathDFS (seekertargetDFS.position, targetDFS.position);
		Debug.Log("DFS ===> Time -> "+timerDFS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedDFS+" (Memory cost -> "+totalMemoryUsedDFS*8+"B for 32b systems and "+totalMemoryUsedDFS*16+"B for 64b systems)");
		FindPathUCS (seekertargetUCS.position, targetUCS.position);
		Debug.Log("UCS ===> Time -> "+timerUCS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedUCS+" (Memory cost -> "+totalMemoryUsedUCS*8+"B for 32b systems and "+totalMemoryUsedUCS*16+"B for 64b systems)");

	*/
	}
	void Start() { 

		FindPath (seekertargetAstar.position, targetAstar.position);
		Debug.Log("A* ===> Time -> "+timerAstar.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedAstar+" (Memory cost -> "+totalMemoryUsedAstar*8+"B for 32b systems and "+totalMemoryUsedAstar*16+"B for 64b systems)");
		FindPathBFS (seekertargetBFS.position, targetBFS.position);
		Debug.Log("BFS ===> Time -> "+timerBFS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedBFS+" (Memory cost -> "+totalMemoryUsedBFS*8+"B for 32b systems and "+totalMemoryUsedBFS*16+"B for 64b systems)");
		FindPathDFS (seekertargetDFS.position, targetDFS.position);
		Debug.Log("DFS ===> Time -> "+timerDFS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedDFS+" (Memory cost -> "+totalMemoryUsedDFS*8+"B for 32b systems and "+totalMemoryUsedDFS*16+"B for 64b systems)");
		FindPathUCS (seekertargetUCS.position, targetUCS.position);
		Debug.Log("UCS ===> Time -> "+timerUCS.Elapsed.ToString()+" & Expanded nodes -> "+totalMemoryUsedUCS+" (Memory cost -> "+totalMemoryUsedUCS*8+"B for 32b systems and "+totalMemoryUsedUCS*16+"B for 64b systems)");

	}

	//A*
	void FindPath(Vector3 startPos, Vector3 targetPos) {
		
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerAstar.Start();
		openSet.Add(startNode);
        
		
		while (openSet.Count > 0) {
			
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				
				RetracePath(startNode,targetNode);
				timerAstar.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode); 
					//heuristic doubling hCost
					//neighbour.hCost = 2*GetDistance(neighbour, targetNode); 
					//heuristic with hCost = gCost
					//neighbour.hCost = GetDistanceUCS(neighbour, targetNode); 
					neighbour.parent = node;
					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		timerAstar.Stop();
		
	}
     
 	//BFS
	void FindPathBFS(Vector3 startPos, Vector3 targetPos) { 

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

    	Queue<Node> queueBFS = new Queue<Node>();
	  	HashSet<Node> closedSet = new HashSet<Node>();

		timerBFS.Start();
		queueBFS.Enqueue(startNode);
        
		while(queueBFS.Count!=0) { 
			Node currentNode= queueBFS.Dequeue();
			if (currentNode == targetNode) {
				RetracePathBFS(startNode,targetNode);
				timerBFS.Stop();
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !queueBFS.Contains(neighbour)) {
					closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					queueBFS.Enqueue(neighbour);
				}
		}
		}
		timerBFS.Stop();
	}

	//DFS
	void FindPathDFS(Vector3 startPos, Vector3 targetPos) { 

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

      	Stack<Node> StackDFS = new Stack<Node>();
	  	HashSet<Node> closedSet = new HashSet<Node>();

	    timerDFS.Start();
		StackDFS.Push(startNode);
        
		while(StackDFS.Count!=0) { 
			Node currentNode= StackDFS.Pop();
			if (currentNode == targetNode) {
				
				RetracePathDFS(startNode,targetNode);
				timerDFS.Stop();
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !StackDFS.Contains(neighbour)) {
					closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					StackDFS.Push(neighbour);
				}
		}
		}
		timerDFS.Stop();
	}

	void RetracePath(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;
		
		while (currentNode != startNode) {
			totalMemoryUsedAstar=totalMemoryUsedAstar+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.path = path;

	}
    void RetracePathUCS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedUCS=totalMemoryUsedUCS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathUCS = path;

	}
    void RetracePathBFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedBFS=totalMemoryUsedBFS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathBFS = path;

	}
    void RetracePathDFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedDFS=totalMemoryUsedDFS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathDFS = path;

	}
	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}
	int GetSameDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 10*dstY + 10* (dstX-dstY);
		return 10*dstX + 10 * (dstY-dstX);
	}
	int GetDistanceUCS(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		/*if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);*/  //same distance as A* gcost

		return (int) Mathf.Sqrt((Mathf.Pow(dstX, 2) + Mathf.Pow(dstY, 2))); //normal straight line distance same in all directions
		
	}

	//UCS
	void FindPathUCS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerUCS.Start();
		openSet.Add(startNode);
		
		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].gCost < node.gCost || openSet[i].gCost == node.gCost) {
					
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				
				RetracePathUCS(startNode,targetNode);
				timerUCS.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceUCS(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		timerUCS.Stop();
	}
}